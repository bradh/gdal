add_gdal_driver(TARGET gdal_HEIF SOURCES heifdataset.cpp st0601.cpp st0601.h PLUGIN_CAPABLE)
gdal_standard_includes(gdal_HEIF)
gdal_target_link_libraries(gdal_HEIF PRIVATE HEIF::HEIF)
