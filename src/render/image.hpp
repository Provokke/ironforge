// 8-bit RGB images plus a dependency-free PNG encoder (zlib "stored" blocks,
// no compression) and a matching decoder for the subset it writes.
#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ifg {

struct Image {
    int width = 0, height = 0;
    std::vector<std::uint8_t> rgb;  // width*height*3, row-major, top row first
    Image() = default;
    Image(int w, int h) : width(w), height(h), rgb(static_cast<std::size_t>(w) * h * 3, 0) {}
    std::uint8_t* px(int x, int y) { return &rgb[(static_cast<std::size_t>(y) * width + x) * 3]; }
    const std::uint8_t* px(int x, int y) const { return &rgb[(static_cast<std::size_t>(y) * width + x) * 3]; }
};

std::uint32_t crc32(const std::uint8_t* data, std::size_t n, std::uint32_t crc = 0);
std::uint32_t adler32(const std::uint8_t* data, std::size_t n);
std::uint64_t fnv1a64(const std::uint8_t* data, std::size_t n);

std::vector<std::uint8_t> encode_png(const Image& img);
std::optional<Image> decode_png(const std::vector<std::uint8_t>& bytes);  // stored-deflate, filter 0, RGB8 only
bool write_file(const std::string& path, const std::vector<std::uint8_t>& bytes);
std::optional<std::vector<std::uint8_t>> read_file(const std::string& path);
bool write_ppm(const std::string& path, const Image& img);

struct ImageDiff {
    std::size_t differing_pixels = 0;  // pixels whose max channel delta > tolerance
    int max_delta = 0;
    double mean_abs_error = 0;
};
ImageDiff compare_images(const Image& a, const Image& b, int tolerance);

}  // namespace ifg
