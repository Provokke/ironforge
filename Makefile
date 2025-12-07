.PHONY: all test sanitize bench demo clean
all:
	cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build -j2
test:
	./scripts/test.sh
sanitize:
	./scripts/sanitize.sh
bench: all
	./build/ironforge_bench
demo: all
	./build/ironforge_demo demo.png 4
clean:
	rm -rf build build-asan build-tsan
