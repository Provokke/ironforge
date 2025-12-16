#include "render/image.hpp"

#include <cstdlib>
#include <cstring>
#include <fstream>

namespace ifg {

std::uint32_t crc32(const std::uint8_t* data, std::size_t n, std::uint32_t crc) {
    static std::uint32_t table[256];
    static bool init = [] {
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
        return true;
    }();
    (void)init;
    crc = ~crc;
    for (std::size_t i = 0; i < n; ++i) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

std::uint32_t adler32(const std::uint8_t* data, std::size_t n) {
    std::uint32_t a = 1, b = 0;
    for (std::size_t i = 0; i < n; ++i) {
        a = (a + data[i]) % 65521;
        b = (b + a) % 65521;
    }
    return (b << 16) | a;
}

std::uint64_t fnv1a64(const std::uint8_t* data, std::size_t n) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (std::size_t i = 0; i < n; ++i) {
        h ^= data[i];
        h *= 0x100000001b3ull;
    }
    return h;
}

static void put32(std::vector<std::uint8_t>& v, std::uint32_t x) {
    v.push_back(std::uint8_t(x >> 24));
    v.push_back(std::uint8_t(x >> 16));
    v.push_back(std::uint8_t(x >> 8));
    v.push_back(std::uint8_t(x));
}
static std::uint32_t get32(const std::uint8_t* p) {
    return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) | (std::uint32_t(p[2]) << 8) | p[3];
}

static void chunk(std::vector<std::uint8_t>& out, const char* type, const std::vector<std::uint8_t>& data) {
    put32(out, static_cast<std::uint32_t>(data.size()));
    std::size_t start = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), data.begin(), data.end());
    put32(out, crc32(&out[start], out.size() - start));
}

std::vector<std::uint8_t> encode_png(const Image& img) {
    std::vector<std::uint8_t> out = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    std::vector<std::uint8_t> ihdr;
    put32(ihdr, static_cast<std::uint32_t>(img.width));
    put32(ihdr, static_cast<std::uint32_t>(img.height));
    ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0});  // 8-bit, truecolor RGB, deflate, filter 0, no interlace
    chunk(out, "IHDR", ihdr);

    // raw scanlines, each prefixed with filter type 0
    std::vector<std::uint8_t> raw;
    std::size_t stride = static_cast<std::size_t>(img.width) * 3;
    raw.reserve((stride + 1) * img.height);
    for (int y = 0; y < img.height; ++y) {
        raw.push_back(0);
        raw.insert(raw.end(), img.rgb.begin() + static_cast<std::ptrdiff_t>(y * stride),
                   img.rgb.begin() + static_cast<std::ptrdiff_t>((y + 1) * stride));
    }
    std::vector<std::uint8_t> z = {0x78, 0x01};
    std::size_t pos = 0;
    do {
        std::size_t len = std::min<std::size_t>(65535, raw.size() - pos);
        bool last = pos + len == raw.size();
        z.push_back(last ? 1 : 0);
        z.push_back(std::uint8_t(len));
        z.push_back(std::uint8_t(len >> 8));
        z.push_back(std::uint8_t(~len));
        z.push_back(std::uint8_t(~len >> 8));
        z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos), raw.begin() + static_cast<std::ptrdiff_t>(pos + len));
        pos += len;
    } while (pos < raw.size());
    put32(z, adler32(raw.data(), raw.size()));
    chunk(out, "IDAT", z);
    chunk(out, "IEND", {});
    return out;
}

std::optional<Image> decode_png(const std::vector<std::uint8_t>& b) {
    static const std::uint8_t sig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    if (b.size() < 8 || std::memcmp(b.data(), sig, 8) != 0) return std::nullopt;
    std::size_t p = 8;
    int w = 0, h = 0;
    std::vector<std::uint8_t> z;
    while (p + 12 <= b.size()) {
        std::uint32_t len = get32(&b[p]);
        if (p + 12 + len > b.size()) return std::nullopt;
        const std::uint8_t* type = &b[p + 4];
        const std::uint8_t* data = &b[p + 8];
        if (crc32(type, len + 4) != get32(data + len)) return std::nullopt;
        if (!std::memcmp(type, "IHDR", 4)) {
            w = static_cast<int>(get32(data));
            h = static_cast<int>(get32(data + 4));
            if (data[8] != 8 || data[9] != 2 || data[12] != 0) return std::nullopt;
        } else if (!std::memcmp(type, "IDAT", 4)) {
            z.insert(z.end(), data, data + len);
        } else if (!std::memcmp(type, "IEND", 4)) {
            break;
        }
        p += 12 + len;
    }
    if (w <= 0 || h <= 0 || z.size() < 6) return std::nullopt;
    std::vector<std::uint8_t> raw;
    std::size_t q = 2;
    for (;;) {
        if (q + 5 > z.size()) return std::nullopt;
        std::uint8_t hdr = z[q];
        if ((hdr >> 1) != 0) return std::nullopt;  // only stored blocks supported
        std::size_t len = z[q + 1] | (z[q + 2] << 8);
        std::size_t nlen = z[q + 3] | (z[q + 4] << 8);
        if ((len ^ 0xFFFF) != nlen || q + 5 + len > z.size()) return std::nullopt;
        raw.insert(raw.end(), z.begin() + static_cast<std::ptrdiff_t>(q + 5), z.begin() + static_cast<std::ptrdiff_t>(q + 5 + len));
        q += 5 + len;
        if (hdr & 1) break;
    }
    if (q + 4 > z.size() || get32(&z[q]) != adler32(raw.data(), raw.size())) return std::nullopt;
    std::size_t stride = static_cast<std::size_t>(w) * 3;
    if (raw.size() != (stride + 1) * h) return std::nullopt;
    Image img(w, h);
    for (int y = 0; y < h; ++y) {
        if (raw[y * (stride + 1)] != 0) return std::nullopt;
        std::memcpy(&img.rgb[y * stride], &raw[y * (stride + 1) + 1], stride);
    }
    return img;
}

bool write_file(const std::string& path, const std::vector<std::uint8_t>& bytes) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(f);
}

std::optional<std::vector<std::uint8_t>> read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return std::nullopt;
    return std::vector<std::uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

bool write_ppm(const std::string& path, const Image& img) {
    std::string hdr = "P6\n" + std::to_string(img.width) + " " + std::to_string(img.height) + "\n255\n";
    std::vector<std::uint8_t> bytes(hdr.begin(), hdr.end());
    bytes.insert(bytes.end(), img.rgb.begin(), img.rgb.end());
    return write_file(path, bytes);
}

ImageDiff compare_images(const Image& a, const Image& b, int tolerance) {
    ImageDiff d;
    if (a.width != b.width || a.height != b.height) {
        d.differing_pixels = static_cast<std::size_t>(-1);
        return d;
    }
    double sum = 0;
    for (std::size_t i = 0; i < a.rgb.size(); i += 3) {
        int m = 0;
        for (int c = 0; c < 3; ++c) {
            int delta = std::abs(int(a.rgb[i + c]) - int(b.rgb[i + c]));
            m = std::max(m, delta);
            sum += delta;
        }
        d.max_delta = std::max(d.max_delta, m);
        if (m > tolerance) ++d.differing_pixels;
    }
    d.mean_abs_error = a.rgb.empty() ? 0 : sum / double(a.rgb.size());
    return d;
}

}  // namespace ifg
