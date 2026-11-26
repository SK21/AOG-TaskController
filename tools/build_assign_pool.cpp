// Writes the VT object pool as AOG-TaskController uploads it (AOG_TC.iop plus the generated rate source
// page) to the given file, so iop_validator can check the generated objects.
#include "AOG_TC_iop_data.hpp"
#include "assign_page.hpp"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
	int status = 1;
	if (2 != argc)
	{
		std::fprintf(stderr, "usage: build_assign_pool <output .iop>\n");
		status = 2;
	}
	else
	{
		std::vector<std::uint8_t> pool(std::begin(AOG_TC_IOP_DATA), std::end(AOG_TC_IOP_DATA));
		const std::size_t designerSize = pool.size();
		std::string error;
		if (!assign_page::add_to_pool(pool, error))
		{
			std::fprintf(stderr, "FAIL: %s\n", error.c_str());
		}
		else
		{
			std::ofstream out(argv[1], std::ios::binary);
			out.write(reinterpret_cast<const char *>(pool.data()), static_cast<std::streamsize>(pool.size()));
			if (out.good())
			{
				std::printf("OK: %s, %zu bytes (%zu generated)\n", argv[1], pool.size(), pool.size() - designerSize);
				status = 0;
			}
			else
			{
				std::fprintf(stderr, "FAIL: could not write %s\n", argv[1]);
			}
		}
	}
	return status;
}
