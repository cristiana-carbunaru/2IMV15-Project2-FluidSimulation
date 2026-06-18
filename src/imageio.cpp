#include "imageio.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#ifdef IMAGEIO_USE_LIBPNG
#include <png.h>
#endif

// Call this function if there was an error while loading.
static unsigned char *_loadImgError(int *width, int *height)
{
    if (width) *width = -1;
    if (height) *height = -1;
    return nullptr;
}

static bool _endsWith(const char *s, const char *postfix)
{
    if (!s || !postfix) return false;
    const size_t sLen = std::strlen(s);
    const size_t postfixLen = std::strlen(postfix);
    if (postfixLen > sLen) return false;
    return std::strcmp(s + sLen - postfixLen, postfix) == 0;
}

#ifdef IMAGEIO_USE_LIBPNG

static unsigned char *_loadImageRGBApng(char *fileName, int *width, int *height)
{
    FILE *fp = std::fopen(fileName, "rb");
    if (!fp) return _loadImgError(width, height);

    const int HEADER_LENGTH = 8;
    png_byte header[HEADER_LENGTH];
    if (std::fread(header, 1, HEADER_LENGTH, fp) != HEADER_LENGTH ||
        png_sig_cmp(header, 0, HEADER_LENGTH))
    {
        std::fclose(fp);
        return _loadImgError(width, height);
    }

    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr)
    {
        std::fclose(fp);
        return _loadImgError(width, height);
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
    {
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        std::fclose(fp);
        return _loadImgError(width, height);
    }

    png_infop end_info = png_create_info_struct(png_ptr);
    if (!end_info)
    {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        std::fclose(fp);
        return _loadImgError(width, height);
    }

    if (setjmp(png_jmpbuf(png_ptr)))
    {
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        std::fclose(fp);
        return _loadImgError(width, height);
    }

    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, HEADER_LENGTH);

    png_read_info(png_ptr, info_ptr);
    *width = static_cast<int>(png_get_image_width(png_ptr, info_ptr));
    *height = static_cast<int>(png_get_image_height(png_ptr, info_ptr));
    const int bit_depth = png_get_bit_depth(png_ptr, info_ptr);
    const png_byte color_type = png_get_color_type(png_ptr, info_ptr);

    // Convert every supported PNG input into RGBA, 8 bits per channel.
    if (color_type != PNG_COLOR_TYPE_RGBA) png_set_expand(png_ptr);
    if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
        png_set_gray_to_rgb(png_ptr);
    if (bit_depth < 8)
        png_set_packing(png_ptr);
    else if (bit_depth == 16)
        png_set_strip_16(png_ptr);
    if (color_type != PNG_COLOR_TYPE_RGBA)
        png_set_filler(png_ptr, 255, PNG_FILLER_AFTER);

    png_read_update_info(png_ptr, info_ptr);

    if (png_get_rowbytes(png_ptr, info_ptr) != static_cast<png_size_t>((*width) * 4))
    {
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        std::fclose(fp);
        return _loadImgError(width, height);
    }

    unsigned char *buffer = static_cast<unsigned char *>(std::malloc((*width) * (*height) * 4));
    if (!buffer)
    {
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        std::fclose(fp);
        return _loadImgError(width, height);
    }

    std::vector<png_bytep> row_pointers(*height);
    for (int y = 0; y < *height; ++y)
        row_pointers[y] = reinterpret_cast<png_bytep>(buffer + ((*height) - 1 - y) * (*width) * 4);

    png_read_image(png_ptr, row_pointers.data());

    std::fclose(fp);
    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
    return buffer;
}

static bool _saveImageRGBApng(char *fileName, unsigned char *buffer, int width, int height)
{
    FILE *fp = std::fopen(fileName, "wb");
    if (!fp) return false;

    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr)
    {
        std::fclose(fp);
        return false;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
    {
        png_destroy_write_struct(&png_ptr, nullptr);
        std::fclose(fp);
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr)))
    {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        std::fclose(fp);
        return false;
    }

    png_init_io(png_ptr, fp);
    png_set_IHDR(png_ptr, info_ptr,
                 width, height, 8, PNG_COLOR_TYPE_RGB_ALPHA,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png_ptr, info_ptr);

    // OpenGL returns screenshots bottom-to-top.  PNG expects top-to-bottom.
    std::vector<png_bytep> row_pointers(height);
    for (int y = 0; y < height; ++y)
        row_pointers[y] = reinterpret_cast<png_bytep>(buffer + (height - 1 - y) * width * 4);

    png_write_image(png_ptr, row_pointers.data());
    png_write_end(png_ptr, info_ptr);

    std::fclose(fp);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return true;
}

#else

static void writeU32BE(std::vector<unsigned char> &out, unsigned int value)
{
    out.push_back(static_cast<unsigned char>((value >> 24) & 0xff));
    out.push_back(static_cast<unsigned char>((value >> 16) & 0xff));
    out.push_back(static_cast<unsigned char>((value >> 8) & 0xff));
    out.push_back(static_cast<unsigned char>(value & 0xff));
}

static unsigned int readU32BE(const unsigned char *p)
{
    return (static_cast<unsigned int>(p[0]) << 24) |
           (static_cast<unsigned int>(p[1]) << 16) |
           (static_cast<unsigned int>(p[2]) << 8) |
           static_cast<unsigned int>(p[3]);
}

static unsigned int crc32Bytes(const unsigned char *data, size_t length)
{
    unsigned int crc = 0xffffffffu;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k)
            crc = (crc & 1u) ? (0xedb88320u ^ (crc >> 1)) : (crc >> 1);
    }
    return crc ^ 0xffffffffu;
}

static unsigned int adler32Bytes(const unsigned char *data, size_t length)
{
    const unsigned int MOD_ADLER = 65521u;
    unsigned int a = 1u;
    unsigned int b = 0u;
    for (size_t i = 0; i < length; ++i)
    {
        a = (a + data[i]) % MOD_ADLER;
        b = (b + a) % MOD_ADLER;
    }
    return (b << 16) | a;
}

static void appendChunk(std::vector<unsigned char> &png,
                        const char type[4],
                        const std::vector<unsigned char> &data)
{
    writeU32BE(png, static_cast<unsigned int>(data.size()));

    const size_t typeOffset = png.size();
    png.push_back(static_cast<unsigned char>(type[0]));
    png.push_back(static_cast<unsigned char>(type[1]));
    png.push_back(static_cast<unsigned char>(type[2]));
    png.push_back(static_cast<unsigned char>(type[3]));
    png.insert(png.end(), data.begin(), data.end());

    const unsigned int crc = crc32Bytes(&png[typeOffset], 4 + data.size());
    writeU32BE(png, crc);
}

static std::vector<unsigned char> zlibStoreOnly(const std::vector<unsigned char> &raw)
{
    std::vector<unsigned char> z;

    z.push_back(0x78);
    z.push_back(0x01);

    size_t offset = 0;
    while (offset < raw.size())
    {
        const size_t remaining = raw.size() - offset;
        const unsigned int blockLen = static_cast<unsigned int>(remaining > 65535 ? 65535 : remaining);
        const bool finalBlock = (offset + blockLen == raw.size());

        // DEFLATE stored block header.  BTYPE=00 means no compression.
        z.push_back(finalBlock ? 0x01 : 0x00);
        z.push_back(static_cast<unsigned char>(blockLen & 0xff));
        z.push_back(static_cast<unsigned char>((blockLen >> 8) & 0xff));
        const unsigned int nlen = 0xffffu - blockLen;
        z.push_back(static_cast<unsigned char>(nlen & 0xff));
        z.push_back(static_cast<unsigned char>((nlen >> 8) & 0xff));

        z.insert(z.end(), raw.begin() + static_cast<long>(offset), raw.begin() + static_cast<long>(offset + blockLen));
        offset += blockLen;
    }

    const unsigned int adler = adler32Bytes(raw.data(), raw.size());
    writeU32BE(z, adler);
    return z;
}

static bool inflateStoreOnly(const std::vector<unsigned char> &z, std::vector<unsigned char> &raw)
{
    if (z.size() < 6) return false;
    size_t pos = 2; // skip zlib header

    bool finalBlock = false;
    while (!finalBlock)
    {
        if (pos + 5 > z.size()) return false;
        const unsigned char header = z[pos++];
        finalBlock = (header & 0x01u) != 0;

        // We only support stored blocks (BTYPE=00), which is what our writer uses.
        if ((header & 0x06u) != 0) return false;

        const unsigned int len = static_cast<unsigned int>(z[pos]) |
                                 (static_cast<unsigned int>(z[pos + 1]) << 8);
        const unsigned int nlen = static_cast<unsigned int>(z[pos + 2]) |
                                  (static_cast<unsigned int>(z[pos + 3]) << 8);
        pos += 4;
        if (((len ^ 0xffffu) & 0xffffu) != nlen) return false;
        if (pos + len > z.size()) return false;

        raw.insert(raw.end(), z.begin() + static_cast<long>(pos), z.begin() + static_cast<long>(pos + len));
        pos += len;
    }

    return pos + 4 <= z.size();
}

static bool _saveImageRGBApng(char *fileName, unsigned char *buffer, int width, int height)
{
    if (!fileName || !buffer || width <= 0 || height <= 0) return false;

    std::vector<unsigned char> raw;
    raw.reserve(static_cast<size_t>(height) * (1 + static_cast<size_t>(width) * 4));

    for (int y = 0; y < height; ++y)
    {
        raw.push_back(0); // PNG filter type 0: no filtering for this scanline.
        const unsigned char *row = buffer + static_cast<size_t>(height - 1 - y) * width * 4;
        raw.insert(raw.end(), row, row + static_cast<size_t>(width) * 4);
    }

    std::vector<unsigned char> png;
    const unsigned char signature[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    png.insert(png.end(), signature, signature + 8);

    std::vector<unsigned char> ihdr;
    writeU32BE(ihdr, static_cast<unsigned int>(width));
    writeU32BE(ihdr, static_cast<unsigned int>(height));
    ihdr.push_back(8); // bit depth
    ihdr.push_back(6); // color type: RGBA
    ihdr.push_back(0); // compression method
    ihdr.push_back(0); // filter method
    ihdr.push_back(0); // no interlace
    appendChunk(png, "IHDR", ihdr);

    const std::vector<unsigned char> idat = zlibStoreOnly(raw);
    appendChunk(png, "IDAT", idat);

    const std::vector<unsigned char> empty;
    appendChunk(png, "IEND", empty);

    FILE *fp = std::fopen(fileName, "wb");
    if (!fp) return false;
    const size_t written = std::fwrite(png.data(), 1, png.size(), fp);
    std::fclose(fp);
    return written == png.size();
}

static unsigned char *_loadImageRGBApng(char *fileName, int *width, int *height)
{
    FILE *fp = std::fopen(fileName, "rb");
    if (!fp) return _loadImgError(width, height);

    std::fseek(fp, 0, SEEK_END);
    const long fileSize = std::ftell(fp);
    std::fseek(fp, 0, SEEK_SET);
    if (fileSize <= 0)
    {
        std::fclose(fp);
        return _loadImgError(width, height);
    }

    std::vector<unsigned char> file(static_cast<size_t>(fileSize));
    if (std::fread(file.data(), 1, file.size(), fp) != file.size())
    {
        std::fclose(fp);
        return _loadImgError(width, height);
    }
    std::fclose(fp);

    const unsigned char signature[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    if (file.size() < 8 || std::memcmp(file.data(), signature, 8) != 0)
        return _loadImgError(width, height);

    int w = -1;
    int h = -1;
    std::vector<unsigned char> idat;
    size_t pos = 8;
    while (pos + 12 <= file.size())
    {
        const unsigned int length = readU32BE(&file[pos]);
        pos += 4;
        if (pos + 4 + length + 4 > file.size()) return _loadImgError(width, height);

        const char type[5] = {
            static_cast<char>(file[pos]), static_cast<char>(file[pos + 1]),
            static_cast<char>(file[pos + 2]), static_cast<char>(file[pos + 3]), 0
        };
        pos += 4;

        const unsigned char *chunkData = &file[pos];
        if (std::strcmp(type, "IHDR") == 0)
        {
            if (length != 13) return _loadImgError(width, height);
            w = static_cast<int>(readU32BE(chunkData));
            h = static_cast<int>(readU32BE(chunkData + 4));
            const unsigned char bitDepth = chunkData[8];
            const unsigned char colorType = chunkData[9];
            const unsigned char compression = chunkData[10];
            const unsigned char filter = chunkData[11];
            const unsigned char interlace = chunkData[12];
            if (w <= 0 || h <= 0 || bitDepth != 8 || colorType != 6 ||
                compression != 0 || filter != 0 || interlace != 0)
                return _loadImgError(width, height);
        }
        else if (std::strcmp(type, "IDAT") == 0)
        {
            idat.insert(idat.end(), chunkData, chunkData + length);
        }
        else if (std::strcmp(type, "IEND") == 0)
        {
            break;
        }

        pos += length + 4; // skip chunk payload and CRC
    }

    if (w <= 0 || h <= 0 || idat.empty()) return _loadImgError(width, height);

    std::vector<unsigned char> raw;
    if (!inflateStoreOnly(idat, raw)) return _loadImgError(width, height);

    const size_t stride = 1 + static_cast<size_t>(w) * 4;
    if (raw.size() != static_cast<size_t>(h) * stride) return _loadImgError(width, height);

    unsigned char *buffer = static_cast<unsigned char *>(std::malloc(static_cast<size_t>(w) * h * 4));
    if (!buffer) return _loadImgError(width, height);

    for (int y = 0; y < h; ++y)
    {
        const unsigned char filter = raw[static_cast<size_t>(y) * stride];
        if (filter != 0)
        {
            std::free(buffer);
            return _loadImgError(width, height);
        }

        const unsigned char *src = &raw[static_cast<size_t>(y) * stride + 1];
        unsigned char *dst = buffer + static_cast<size_t>(h - 1 - y) * w * 4;
        std::memcpy(dst, src, static_cast<size_t>(w) * 4);
    }

    *width = w;
    *height = h;
    return buffer;
}

#endif

// Public interface used by the simulator:

unsigned char *loadImageRGBA(char *fileName, int *width, int *height)
{
    if (_endsWith(fileName, ".png"))
        return _loadImageRGBApng(fileName, width, height);
    return _loadImgError(width, height);
}

bool saveImageRGBA(char *fileName, unsigned char *buffer, int width, int height)
{
    if (_endsWith(fileName, ".png"))
        return _saveImageRGBApng(fileName, buffer, width, height);
    return false;
}
