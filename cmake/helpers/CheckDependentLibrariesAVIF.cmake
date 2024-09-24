define_find_package2(AVIF avif/avif.h avif PKGCONFIG_NAME libavif)
gdal_check_package(AVIF "AVIF" CAN_DISABLE)

# Check for compile, link, run compatibility.
include(CheckCXXSourceCompiles)
check_cxx_source_compiles(
    "
    #include <avif/avif.h>
    int main()
    {
        avifImage *image;
        image->numProperties;
        return 0;
    }
    "
    AVIF_HAS_OPAQUE_PROPERTIES
)
if (AVIF_HAS_OPAQUE_PROPERTIES)
    set(AVIF_HAS_OPAQUE_PROPERTIES 1)
    add_definitions(-DAVIF_HAS_OPAQUE_PROPERTIES)
endif ()