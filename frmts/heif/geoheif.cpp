/******************************************************************************
 * Project:  GeoHEIF support class
 * SPDX-License-Identifier: MIT
 ****************************************************************************/
#include "geoheif.h"

GeoHEIF::GeoHEIF()
{
}

GeoHEIF::~GeoHEIF()
{
}

bool GeoHEIF::has_SRS() const
{
    return !m_oSRS.IsEmpty();
}

bool GeoHEIF::has_GCPs() const
{
    return haveGCPs;
}

static double to_double(uint8_t *data, uint32_t index)
{
    uint64_t v = 0;
    v |= ((uint64_t)data[index]) << 56;
    v |= ((uint64_t)data[index + 1]) << 48;
    v |= ((uint64_t)data[index + 2]) << 40;
    v |= ((uint64_t)data[index + 3]) << 32;
    v |= ((uint64_t)data[index + 4]) << 24;
    v |= ((uint64_t)data[index + 5]) << 16;
    v |= ((uint64_t)data[index + 6]) << 8;
    v |= ((uint64_t)data[index + 7]) << 0;

    double d = 0;
    memcpy(&d, &v, sizeof(d));
    return d;
}

static double int_as_double(uint8_t *data, uint32_t index)
{
    uint32_t v = 0;
    v |= ((uint64_t)data[index + 0]) << 24;
    v |= ((uint64_t)data[index + 1]) << 16;
    v |= ((uint64_t)data[index + 2]) << 8;
    v |= ((uint64_t)data[index + 3]) << 0;
    return (double)v;
}

CPLErr GeoHEIF::GetGeoTransform(std::shared_ptr<std::vector<uint8_t>> data, double *padfTransform)
{
    // TODO: cache transform
    // Match version
    if (data->data()[0] == 0x00)
    {
        uint32_t index = 0;
        if (data->data()[index + 3] == 0x01)
        {
            index += 4;
            padfTransform[1] = to_double(data->data(), index);
            index += 8;
            padfTransform[2] = to_double(data->data(), index);
            index += 8;
            padfTransform[0] = to_double(data->data(), index);
            index += 8;
            padfTransform[4] = to_double(data->data(), index);
            index += 8;
            padfTransform[5] = to_double(data->data(), index);
            index += 8;
            padfTransform[3] = to_double(data->data(), index);
            return CE_None;
        }
    }
    return CE_Failure;
}

/************************************************************************/
/*                          GetSpatialRef()                             */
/************************************************************************/
const OGRSpatialReference *GeoHEIF::GetSpatialRef() const
{
    return &m_oSRS;
}

/************************************************************************/
/*                          GetSpatialRef()                             */
/************************************************************************/
const OGRSpatialReference *GeoHEIF::GetSpatialRef(std::shared_ptr<std::vector<uint8_t>> data) const
{
    if (!m_oSRS.IsEmpty())
        return &m_oSRS;

    if (data->data()[0] == 0x00)
    {
        extractSRS(data->data(), data->size());
    }
    return &m_oSRS;
}

void GeoHEIF::extractSRS(const uint8_t *payload, size_t length) const
{
    // TODO: more sophisticated length checks
    if (length < 6)
    {
        return;
    }
    std::string crsEncoding(payload + 4, payload + 8);
    std::string crs(payload + 8, payload + length);
    std::cout << "crs: " << crs << std::endl;
    if (crsEncoding == "wkt2")
    {
        m_oSRS.importFromWkt(crs.c_str());
    }
    else if (crsEncoding == "crsu")
    {
        m_oSRS.importFromCRSURL(crs.c_str());
    }
    else if (crsEncoding == "curi")
    {
        if ((crs.at(0) != '[') || (crs.at(crs.length() -1) != ']')) {
            return;
        }
        std::cout << "safe CURIE: " << crs << std::endl;
        std::string curie = crs.substr(1, crs.length() - 2);
        std::cout << "curie: " << curie << std::endl;
        std::string authority = "EPSG"; // TODO
        std::string code = "32755"; // TODO
        std::string osURL("http://www.opengis.net/def/crs/");
        osURL.append(authority);
        osURL += "/0/";
        osURL.append(code);
        m_oSRS.importFromCRSURL(osURL.c_str());
    }
    else
    {
        return;
    }
}

int GeoHEIF::GetGCPCount(std::shared_ptr<std::vector<uint8_t>> data)
{
    if (!haveGCPs)
    {
        return 0;
    }
    if (gcps.size() == 0)
    {
        if (data->data()[0] == 0x00)
        {
            uint32_t index = 0;
            bool is_3D = (data->data()[index + 3] == 0x00);
            index += 4;
            uint16_t count =
                (data->data()[index] << 8) + (data->data()[index + 1]);
            index += 2;
            for (uint16_t j = 0; j < count; j++)
            {
                GDAL_GCP gcp;
                char szID[32];
                snprintf(szID, sizeof(szID), "%d", j);
                gcp.pszId = CPLStrdup(szID);
                gcp.pszInfo = CPLStrdup("");
                gcp.dfGCPPixel = int_as_double(data->data(), index);
                index += 4;
                gcp.dfGCPLine = int_as_double(data->data(), index);
                index += 4;
                gcp.dfGCPX = to_double(data->data(), index);
                index += 8;
                gcp.dfGCPY = to_double(data->data(), index);
                index += 8;
                if (is_3D)
                {
                    gcp.dfGCPZ = to_double(data->data(), index);
                    index += 8;
                }
                else
                {
                    gcp.dfGCPZ = 0.0;
                }
                gcps.push_back(gcp);
            }
            return (int)gcps.size();
        }
        // if we get to here, the property wasn't found, so no GCPs.
        haveGCPs = false;
    }
    return 0;
}

const GDAL_GCP *GeoHEIF::GetGCPs()
{
    // TODO: check if we have any first, and parse out
    return gcps.data();
}

const OGRSpatialReference *GeoHEIF::GetGCPSpatialRef(std::shared_ptr<std::vector<uint8_t>> data) const
{
    return this->GetSpatialRef(data);
}
