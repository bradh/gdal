/******************************************************************************
 * Project:  GeoHEIF support class
 * SPDX-License-Identifier: MIT
 ****************************************************************************/

#ifndef GEOHEIF_H_INCLUDED_
#define GEOHEIF_H_INCLUDED_

#include "gdal_pam.h"
#include "ogr_spatialref.h"

#include "include_libheif.h"

#include "heifdrivercore.h"

#include <vector>

class GeoHEIF final
{
    mutable OGRSpatialReference m_oSRS{};
    bool haveGCPs = true; // lets be optimistic the first time
    std::vector<GDAL_GCP> gcps;

  public:
    GeoHEIF();
    ~GeoHEIF();

    bool has_SRS() const;
    bool has_GCPs() const;
    const OGRSpatialReference *GetSpatialRef() const;
    const OGRSpatialReference *GetSpatialRef(std::shared_ptr<std::vector<uint8_t>> data) const;
    CPLErr GetGeoTransform(std::shared_ptr<std::vector<uint8_t>>, double *);
    int GetGCPCount(std::shared_ptr<std::vector<uint8_t>>);
    const GDAL_GCP *GetGCPs();
    void extractSRS(const uint8_t *payload, size_t length) const;
    const OGRSpatialReference *GetGCPSpatialRef(std::shared_ptr<std::vector<uint8_t>> data) const;
};
#endif /* GEOHEIF_H_INCLUDED_ */
