/******************************************************************************
 *
 * Project:  AVIF driver
 * Author:   Even Rouault <even.rouault at spatialys.com>
 *
 ******************************************************************************
 * Copyright (c) 2024, Even Rouault <even.rouault at spatialys.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 ****************************************************************************/

#include "gdal_pam.h"
#include "cpl_minixml.h"
#include "cpl_vsi_virtual.h"

#include "avifdrivercore.h"
#include "gdalexif.h"
#include "memdataset.h"

#include <avif/avif.h>

#include <algorithm>
#include <cinttypes>
#include <iomanip>
#include <vector>

constexpr const char *DEFAULT_QUALITY_STR = "60";
constexpr const char *DEFAULT_QUALITY_ALPHA_STR = "100";
constexpr const char *DEFAULT_SPEED_STR = "6";

/************************************************************************/
/*                         GDALAVIFDataset                              */
/************************************************************************/

class GDALAVIFDataset final : public GDALPamDataset
{
    friend class GDALAVIFRasterBand;

    avifDecoder *m_decoder = nullptr;
    bool m_bDecodedDone = false;
    bool m_bDecodedOK = false;
    int m_iPart = 0;
    avifRGBImage m_rgb{};  // memset()' to 0 in constructor

    bool Init(GDALOpenInfo *poOpenInfo);
    bool Decode();

    GDALAVIFDataset(const GDALAVIFDataset &) = delete;
    GDALAVIFDataset &operator=(const GDALAVIFDataset &) = delete;

#ifdef AVIF_HAS_OPAQUE_PROPERTIES
  protected:
    void processProperties();
    void getSRS() const;
    void extractSRS(const uint8_t *payload, size_t length) const;
    void extractModelTransformation(const uint8_t *payload,
                                    size_t length) const;
    void extractTAIClockInfo(const uint8_t *payload, size_t length);
    void extractTAITimestamp(const uint8_t *payload, size_t length);
    void extractUserDescription(const uint8_t *payload, size_t length);
    void extractGCPs(const uint8_t *payload, size_t length);
    void processUUIDProperty(const uint8_t usertype[16], const uint8_t *payload,
                             size_t length);
    mutable OGRSpatialReference m_oSRS{};
    mutable bool m_bHasGeoTransform = false;
    mutable double m_adfGeoTransform[6]{0, 1, 0, 0, 0, 1};
    std::vector<GDAL_GCP> gcps;
#endif

  public:
#ifdef AVIF_HAS_OPAQUE_PROPERTIES
    GDALAVIFDataset() : gcps()
#else
    GDALAVIFDataset()
#endif
    {
        memset(&m_rgb, 0, sizeof(m_rgb));
    }

    ~GDALAVIFDataset();

    static GDALPamDataset *OpenStaticPAM(GDALOpenInfo *poOpenInfo);

    static GDALDataset *Open(GDALOpenInfo *poOpenInfo)
    {
        return OpenStaticPAM(poOpenInfo);
    }

    static GDALDataset *CreateCopy(const char *, GDALDataset *, int,
                                   char **papszOptions,
                                   GDALProgressFunc pfnProgress,
                                   void *pProgressData);
#ifdef AVIF_HAS_OPAQUE_PROPERTIES
    const OGRSpatialReference *GetSpatialRef() const override;
    virtual CPLErr GetGeoTransform(double *) override;
    int GetGCPCount() override;
    const GDAL_GCP *GetGCPs() override;
    const OGRSpatialReference *GetGCPSpatialRef() const override;
#endif
};

/************************************************************************/
/*                       GDALAVIFRasterBand                             */
/************************************************************************/

class GDALAVIFRasterBand final : public MEMRasterBand
{
  public:
    GDALAVIFRasterBand(GDALAVIFDataset *poDSIn, int nBandIn,
                       GDALDataType eDataTypeIn, int nBits);

    GDALColorInterp GetColorInterpretation() override
    {
        if (poDS->GetRasterCount() == 1)
            return GCI_GrayIndex;
        else if (poDS->GetRasterCount() == 2)
            return nBand == 1 ? GCI_GrayIndex : GCI_AlphaBand;
        else
            return static_cast<GDALColorInterp>(GCI_RedBand + nBand - 1);
    }

  protected:
    friend class GDALAVIFDataset;

    CPLErr IReadBlock(int, int, void *) override;
    CPLErr IRasterIO(GDALRWFlag eRWFlag, int nXOff, int nYOff, int nXSize,
                     int nYSize, void *pData, int nBufXSize, int nBufYSize,
                     GDALDataType eBufType, GSpacing nPixelSpaceBuf,
                     GSpacing nLineSpaceBuf,
                     GDALRasterIOExtraArg *psExtraArg) override;

    void SetData(GByte *pabyDataIn, int nPixelOffsetIn, int nLineOffsetIn);
};

/************************************************************************/
/*                           GDALAVIFIO                                 */
/************************************************************************/

class GDALAVIFIO
{
  public:
    explicit GDALAVIFIO(VSIVirtualHandleUniquePtr fpIn);

  private:
    avifIO io{};  // memset()' to 0 in constructor
    VSIVirtualHandleUniquePtr fp{};
    vsi_l_offset nFileSize = 0;
    std::vector<GByte> buffer{};

    static void Destroy(struct avifIO *io);
    static avifResult Read(struct avifIO *io, uint32_t readFlags,
                           uint64_t offset, size_t size, avifROData *out);

    GDALAVIFIO(const GDALAVIFIO &) = delete;
    GDALAVIFIO &operator=(const GDALAVIFIO &) = delete;
};

/************************************************************************/
/*                         ~GDALAVIFDataset()                           */
/************************************************************************/

GDALAVIFDataset::~GDALAVIFDataset()
{
    if (m_decoder)
    {
        avifDecoderDestroy(m_decoder);
        avifRGBImageFreePixels(&m_rgb);
    }
}

/************************************************************************/
/*                        GDALAVIFDataset::Decode()                     */
/************************************************************************/

bool GDALAVIFDataset::Decode()
{
    if (m_bDecodedDone)
        return m_bDecodedOK;
    m_bDecodedDone = true;

    auto avifErr = m_iPart == 0 ? avifDecoderNextImage(m_decoder)
                                : avifDecoderNthImage(m_decoder, m_iPart);
    if (avifErr != AVIF_RESULT_OK)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "avifDecoderNextImage() failed with: %s",
                 avifResultToString(avifErr));
        return false;
    }

    avifRGBImageSetDefaults(&m_rgb, m_decoder->image);

    m_rgb.format = (nBands == 1 || nBands == 3) ? AVIF_RGB_FORMAT_RGB
                                                : AVIF_RGB_FORMAT_RGBA;
    const int nChannels = m_rgb.format == AVIF_RGB_FORMAT_RGB ? 3 : 4;

#if AVIF_VERSION_MAJOR >= 1
    avifErr = avifRGBImageAllocatePixels(&m_rgb);
    if (avifErr != AVIF_RESULT_OK)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "avifRGBImageAllocatePixels() failed with: %s",
                 avifResultToString(avifErr));
        return false;
    }
#else
    avifRGBImageAllocatePixels(&m_rgb);
    if (m_rgb.pixels == nullptr)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "avifRGBImageAllocatePixels() failed");
        return false;
    }
#endif

    avifErr = avifImageYUVToRGB(m_decoder->image, &m_rgb);
    if (avifErr != AVIF_RESULT_OK)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "avifImageYUVToRGB() failed with: %s",
                 avifResultToString(avifErr));
        return false;
    }

    const auto eDT = papoBands[0]->GetRasterDataType();
    const int nDTSize = GDALGetDataTypeSizeBytes(eDT);
    for (int i = 0; i < nBands; ++i)
    {
        const int iAVIFChannel = (nBands == 2 && i == 1) ? 3 : i;
        cpl::down_cast<GDALAVIFRasterBand *>(papoBands[i])
            ->SetData(m_rgb.pixels + iAVIFChannel * nDTSize,
                      nDTSize * nChannels, static_cast<int>(m_rgb.rowBytes));
    }

    m_bDecodedOK = true;
    return m_bDecodedOK;
}

/************************************************************************/
/*                         GDALAVIFRasterBand()                         */
/************************************************************************/

GDALAVIFRasterBand::GDALAVIFRasterBand(GDALAVIFDataset *poDSIn, int nBandIn,
                                       GDALDataType eDataTypeIn, int nBits)
    : MEMRasterBand(poDSIn, nBandIn, nullptr, eDataTypeIn, 0, 0, false)
{
    if (nBits != 8 && nBits != 16)
    {
        GDALRasterBand::SetMetadataItem("NBITS", CPLSPrintf("%d", nBits),
                                        "IMAGE_STRUCTURE");
    }
}

/************************************************************************/
/*                               SetData()                              */
/************************************************************************/

void GDALAVIFRasterBand::SetData(GByte *pabyDataIn, int nPixelOffsetIn,
                                 int nLineOffsetIn)
{
    pabyData = pabyDataIn;
    nPixelOffset = nPixelOffsetIn;
    nLineOffset = nLineOffsetIn;
}

/************************************************************************/
/*                            IReadBlock()                              */
/************************************************************************/

CPLErr GDALAVIFRasterBand::IReadBlock(int nBlockXOff, int nBlockYOff,
                                      void *pImage)
{
    GDALAVIFDataset *poGDS = cpl::down_cast<GDALAVIFDataset *>(poDS);
    if (!poGDS->Decode())
        return CE_Failure;
    return MEMRasterBand::IReadBlock(nBlockXOff, nBlockYOff, pImage);
}

/************************************************************************/
/*                             IRasterIO()                              */
/************************************************************************/

CPLErr GDALAVIFRasterBand::IRasterIO(GDALRWFlag eRWFlag, int nXOff, int nYOff,
                                     int nXSize, int nYSize, void *pData,
                                     int nBufXSize, int nBufYSize,
                                     GDALDataType eBufType,
                                     GSpacing nPixelSpaceBuf,
                                     GSpacing nLineSpaceBuf,
                                     GDALRasterIOExtraArg *psExtraArg)
{
    GDALAVIFDataset *poGDS = cpl::down_cast<GDALAVIFDataset *>(poDS);
    if (!poGDS->Decode())
        return CE_Failure;
    return MEMRasterBand::IRasterIO(eRWFlag, nXOff, nYOff, nXSize, nYSize,
                                    pData, nBufXSize, nBufYSize, eBufType,
                                    nPixelSpaceBuf, nLineSpaceBuf, psExtraArg);
}

/************************************************************************/
/*                       GDALAVIFIO::GDALAVIFIO()                       */
/************************************************************************/

GDALAVIFIO::GDALAVIFIO(VSIVirtualHandleUniquePtr fpIn) : fp(std::move(fpIn))
{
    memset(&io, 0, sizeof(io));
    io.destroy = Destroy;
    io.read = Read;

    fp->Seek(0, SEEK_END);
    nFileSize = fp->Tell();
    fp->Seek(0, SEEK_SET);

    io.sizeHint = std::min<uint64_t>(10 * 1024 * 1024, nFileSize);
}

/************************************************************************/
/*                       GDALAVIFIO::Destroy()                          */
/************************************************************************/

/* static */ void GDALAVIFIO::Destroy(struct avifIO *io)
{
    GDALAVIFIO *gdalIO = reinterpret_cast<GDALAVIFIO *>(io);
    delete gdalIO;
}

/************************************************************************/
/*                       GDALAVIFIO::Read()                             */
/************************************************************************/

/* static */ avifResult GDALAVIFIO::Read(struct avifIO *io, uint32_t readFlags,
                                         uint64_t offset, size_t size,
                                         avifROData *out)
{
    GDALAVIFIO *gdalIO = reinterpret_cast<GDALAVIFIO *>(io);
    if (readFlags != 0)
    {
        // Unsupported readFlags
        return AVIF_RESULT_IO_ERROR;
    }
    if (offset > gdalIO->nFileSize)
    {
        return AVIF_RESULT_IO_ERROR;
    }

    if (offset == gdalIO->nFileSize)
    {
        out->data = gdalIO->buffer.data();
        out->size = 0;
        return AVIF_RESULT_OK;
    }

    const uint64_t availableSize = gdalIO->nFileSize - offset;
    size = static_cast<size_t>(std::min<uint64_t>(size, availableSize));
    try
    {
        gdalIO->buffer.resize(size);
    }
    catch (const std::bad_alloc &)
    {
        CPLError(CE_Failure, CPLE_OutOfMemory,
                 "Out of memory in GDALAVIFIO::Read()");
        return AVIF_RESULT_IO_ERROR;
    }

    if (gdalIO->fp->Seek(offset, SEEK_SET) != 0 ||
        gdalIO->fp->Read(gdalIO->buffer.data(), size, 1) != 1)
    {
        return AVIF_RESULT_IO_ERROR;
    }

    out->data = gdalIO->buffer.data();
    out->size = size;
    return AVIF_RESULT_OK;
}

#ifdef AVIF_HAS_OPAQUE_PROPERTIES
/************************************************************************/
/*                          GetSpatialRef()                             */
/************************************************************************/

const OGRSpatialReference *GDALAVIFDataset::GetSpatialRef() const

{
    if (!m_oSRS.IsEmpty())
    {
        return &m_oSRS;
    }

    getSRS();
    return &m_oSRS;
}

void GDALAVIFDataset::getSRS() const
{
    for (size_t i = 0; i < m_decoder->image->numProperties; i++)
    {
        avifImageItemProperty *prop = &(m_decoder->image->properties[i]);
        if (!memcmp(prop->boxtype, "mcrs", 4))
        {
            extractSRS(prop->boxpayload.data, prop->boxpayload.size);
            break;
        }
    }
}

/************************************************************************/
/*                          GetGeoTransform()                           */
/************************************************************************/

CPLErr GDALAVIFDataset::GetGeoTransform(double *padfTransform)

{
    if (m_bHasGeoTransform)
    {
        memcpy(padfTransform, m_adfGeoTransform, sizeof(double) * 6);
        return CE_None;
    }

    processProperties();

    if (!m_bHasGeoTransform)
    {
        return CE_Failure;
    }

    memcpy(padfTransform, m_adfGeoTransform, sizeof(double) * 6);

    return CE_None;
}

void GDALAVIFDataset::processProperties()
{
    for (size_t i = 0; i < m_decoder->image->numProperties; i++)
    {
        avifImageItemProperty *prop = &(m_decoder->image->properties[i]);
        if (!memcmp(prop->boxtype, "mcrs", 4))
        {
            extractSRS(prop->boxpayload.data, prop->boxpayload.size);
        }
        else if (!memcmp(prop->boxtype, "mtxf", 4))
        {
            extractModelTransformation(prop->boxpayload.data,
                                       prop->boxpayload.size);
        }
        else if (!memcmp(prop->boxtype, "taic", 4))
        {
            extractTAIClockInfo(prop->boxpayload.data, prop->boxpayload.size);
        }
        else if (!memcmp(prop->boxtype, "itai", 4))
        {
            extractTAITimestamp(prop->boxpayload.data, prop->boxpayload.size);
        }
        else if (!memcmp(prop->boxtype, "udes", 4))
        {
            extractUserDescription(prop->boxpayload.data,
                                   prop->boxpayload.size);
        }
        else if (!memcmp(prop->boxtype, "tiep", 4))
        {
            extractGCPs(prop->boxpayload.data, prop->boxpayload.size);
        }
        else if (!memcmp(prop->boxtype, "uuid", 4))
        {
            processUUIDProperty(prop->usertype, prop->boxpayload.data,
                                prop->boxpayload.size);
        }
    }
}

void GDALAVIFDataset::extractSRS(const uint8_t *payload, size_t length) const
{
    // TODO: more sophisticated length checks
    if (length < 6)
    {
        return;
    }
    if ((!memcmp(&(payload[4]), "wkt2", 4)))
    {
        // TODO: make sure its null terminated...
        m_oSRS.importFromWkt(reinterpret_cast<const char *>(&(payload[8])));
    }
    else
    {
        // TODO: add CRS encoding
    }
}

static uint32_t to_uint32(const uint8_t *data, uint32_t index)
{
    uint32_t v = 0;
    v |= static_cast<uint32_t>(data[index]) << 24;
    v |= static_cast<uint32_t>(data[index + 1]) << 16;
    v |= static_cast<uint32_t>(data[index + 2]) << 8;
    v |= static_cast<uint32_t>(data[index + 3]) << 0;
    return v;
}

static int32_t to_int32(const uint8_t *data, uint32_t index)
{
    uint32_t v = to_uint32(data, index);
    int32_t r = 0;
    memcpy(&r, &v, sizeof(v));
    return r;
}

static uint64_t to_uint64(const uint8_t *data, uint32_t index)
{
    uint64_t v = 0;
    v |= static_cast<uint64_t>(data[index]) << 56;
    v |= static_cast<uint64_t>(data[index + 1]) << 48;
    v |= static_cast<uint64_t>(data[index + 2]) << 40;
    v |= static_cast<uint64_t>(data[index + 3]) << 32;
    v |= static_cast<uint64_t>(data[index + 4]) << 24;
    v |= static_cast<uint64_t>(data[index + 5]) << 16;
    v |= static_cast<uint64_t>(data[index + 6]) << 8;
    v |= static_cast<uint64_t>(data[index + 7]) << 0;
    return v;
}

static double to_double(const uint8_t *data, uint32_t index)
{
    uint64_t v = to_uint64(data, index);
    double d = 0;
    memcpy(&d, &v, sizeof(d));
    return d;
}

void GDALAVIFDataset::extractModelTransformation(const uint8_t *payload,
                                                 size_t length) const
{
    // TODO: this only handles the 2D case.
    if (length != 52)
    {
        return;
    }
    // Match version
    if (payload[0] == 0x00)
    {
        uint32_t index = 0;
        if (payload[index + 3] == 0x01)
        {
            index += 4;
            m_adfGeoTransform[1] = to_double(payload, index);
            index += 8;
            m_adfGeoTransform[2] = to_double(payload, index);
            index += 8;
            m_adfGeoTransform[0] = to_double(payload, index);
            index += 8;
            m_adfGeoTransform[4] = to_double(payload, index);
            index += 8;
            m_adfGeoTransform[5] = to_double(payload, index);
            index += 8;
            m_adfGeoTransform[3] = to_double(payload, index);
            m_bHasGeoTransform = true;
        }
    }
}

void GDALAVIFDataset::extractTAIClockInfo(const uint8_t *payload, size_t length)
{
    if (length != 21)
    {
        return;
    }
    // Match version
    if (payload[0] == 0x00)
    {
        uint32_t index = 4;
        uint64_t time_uncertainty = to_uint64(payload, index);
        index += sizeof(uint64_t);
        if (time_uncertainty == UINT64_MAX)
        {
            GDALDataset::SetMetadataItem("TIME_UNCERTAINTY", "(Unknown)",
                                         "TIMING");
        }
        else
        {
            const std::string time_uncertainty_formatted =
                std::to_string(time_uncertainty) + " ns";
            GDALDataset::SetMetadataItem("TIME_UNCERTAINTY",
                                         time_uncertainty_formatted.c_str(),
                                         "TIMING");
        }

        uint32_t clock_resolution = to_uint32(payload, index);
        index += sizeof(uint32_t);
        const std::string clock_resolution_formatted =
            std::to_string(clock_resolution) + " ns";
        GDALDataset::SetMetadataItem(
            "CLOCK_RESOLUTION", clock_resolution_formatted.c_str(), "TIMING");

        int32_t clock_drift_rate = to_int32(payload, index);
        index += sizeof(uint32_t);
        if (clock_drift_rate == INT32_MAX)
        {
            GDALDataset::SetMetadataItem("CLOCK_DRIFT_RATE", "(Unknown)",
                                         "TIMING");
        }
        else
        {
            const std::string clock_drift_rate_formatted =
                std::to_string(clock_drift_rate) + " ps/s";
            GDALDataset::SetMetadataItem("CLOCK_DRIFT_RATE",
                                         clock_drift_rate_formatted.c_str(),
                                         "TIMING");
        }
        uint8_t clock_type = (payload[index] >> 6);
        switch (clock_type)
        {
            case 0:
                GDALDataset::SetMetadataItem("CLOCK_TYPE", "0 (Unknown)",
                                             "TIMING");
                break;
            case 1:
                GDALDataset::SetMetadataItem(
                    "CLOCK_TYPE",
                    "1 (Does not synchronize to absolute TAI time)", "TIMING");
                break;
            case 2:
                GDALDataset::SetMetadataItem(
                    "CLOCK_TYPE", "2 (Can synchronize to absolute TAI time)",
                    "TIMING");
                break;
            default:
                break;
        }
    }
}

void GDALAVIFDataset::extractTAITimestamp(const uint8_t *payload, size_t length)
{
    if (length != 13)
    {
        return;
    }
    // Match version
    if (payload[0] == 0x00)
    {
        uint32_t index = 4;
        uint64_t tai_timestamp = to_uint64(payload, index);
        index += sizeof(uint64_t);
        const std::string tai_timestamp_formatted =
            std::to_string(tai_timestamp) + " ns";
        GDALDataset::SetMetadataItem("TAI_TIMESTAMP",
                                     tai_timestamp_formatted.c_str(), "TIMING");
        uint8_t f = payload[index];
        bool synchronization_state = ((f & 0x80) == 0x80);
        std::string synchronization_state_formatted =
            synchronization_state ? "SYNCHRONIZED" : "NOT SYNCHRONIZED";
        GDALDataset::SetMetadataItem("SYNCHRONIZATION_STATE",
                                     synchronization_state_formatted.c_str(),
                                     "TIMING");
        bool timestamp_generation_failure = ((f & 0x40) == 0x40);
        std::string timestamp_generation_failure_formatted =
            timestamp_generation_failure ? "YES" : "NO";
        GDALDataset::SetMetadataItem(
            "TIMESTAMP_GENERATION_FAILURE",
            timestamp_generation_failure_formatted.c_str(), "TIMING");
        bool timestamp_is_modified = ((f & 0x20) == 0x20);
        std::string timestamp_is_modified_formatted =
            timestamp_is_modified ? "YES" : "NO";
        GDALDataset::SetMetadataItem("TIMESTAMP_IS_MODIFIED",
                                     timestamp_is_modified_formatted.c_str(),
                                     "TIMING");
    }
}

void GDALAVIFDataset::extractUserDescription(const uint8_t *payload,
                                             size_t length)
{
    // Match version
    if (payload[0] == 0x00)
    {
        std::stringstream ss(std::string(payload + 4, payload + length));
        std::string lang;
        std::getline(ss, lang, '\0');
        std::string name;
        std::getline(ss, name, '\0');
        std::string description;
        std::getline(ss, description, '\0');
        std::string tags;
        std::getline(ss, tags, '\0');
        std::string domain = "DESCRIPTION";
        if (!lang.empty())
        {
            domain += "_";
            domain += lang;
        }
        GDALDataset::SetMetadataItem("NAME", name.c_str(), domain.c_str());
        GDALDataset::SetMetadataItem("DESCRIPTION", description.c_str(),
                                     domain.c_str());
        if (!tags.empty())
        {
            GDALDataset::SetMetadataItem("TAGS", tags.c_str(), domain.c_str());
        }
    }
}

void GDALAVIFDataset::extractGCPs(const uint8_t *payload, size_t length)
{
    if (length < 30)
    {
        return;
    }
    // Match version
    if (payload[0] == 0x00)
    {
        uint32_t index = 0;
        bool is_3D = (payload[3] == 0x00);
        index += 4;
        uint16_t count = (payload[index] << 8) + (payload[index + 1]);
        index += 2;
        for (uint16_t j = 0; j < count; j++)
        {
            GDAL_GCP gcp;
            char szID[32];
            snprintf(szID, sizeof(szID), "%d", j);
            gcp.pszId = CPLStrdup(szID);
            gcp.pszInfo = CPLStrdup("");
            gcp.dfGCPPixel = static_cast<double>(to_int32(payload, index));
            index += sizeof(int32_t);
            gcp.dfGCPLine = static_cast<double>(to_int32(payload, index));
            index += sizeof(int32_t);
            gcp.dfGCPX = to_double(payload, index);
            index += sizeof(double);
            gcp.dfGCPY = to_double(payload, index);
            index += sizeof(double);
            if (is_3D)
            {
                gcp.dfGCPZ = to_double(payload, index);
                index += sizeof(double);
            }
            else
            {
                gcp.dfGCPZ = 0.0;
            }
            gcps.push_back(gcp);
        }
    }
}

static std::string formatUUID(uint8_t bytes[16])
{
    std::stringstream canonicalForm;
    canonicalForm << std::hex << std::setw(2);
    for (int i = 0; i < 4; i++)
    {
        canonicalForm << static_cast<int>(bytes[i]);
    }
    canonicalForm << "-";
    canonicalForm << static_cast<int>(bytes[4]);
    canonicalForm << static_cast<int>(bytes[5]);
    canonicalForm << "-";
    canonicalForm << static_cast<int>(bytes[6]);
    canonicalForm << static_cast<int>(bytes[7]);
    canonicalForm << "-";
    canonicalForm << static_cast<int>(bytes[8]);
    canonicalForm << static_cast<int>(bytes[9]);
    canonicalForm << "-";
    for (int i = 10; i < 16; i++)
    {
        canonicalForm << static_cast<int>(bytes[i]);
    }
    return canonicalForm.str();
}

void GDALAVIFDataset::processUUIDProperty(const uint8_t usertype[16],
                                          const uint8_t *payload, size_t length)
{
    if (length != 16)
    {
        return;
    }
    if (!memcmp(
            usertype,
            "\x4a\x66\xef\xa7\xe5\x41\x52\x6c\x94\x27\x9e\x77\x61\x7f\xeb\x7d",
            16))
    {
        uint8_t rawContentID[16];
        memcpy(&rawContentID[0], payload, 16);
        std::string formattedContentID = formatUUID(rawContentID);
        GDALDataset::SetMetadataItem("ITEM_CONTENT_ID",
                                     formattedContentID.c_str(), "GIMI");
    }
    else
    {
        // TODO: more decoding
    }
}

int GDALAVIFDataset::GetGCPCount()
{
    return gcps.size();
}

const GDAL_GCP *GDALAVIFDataset::GetGCPs()
{
    return gcps.data();
}

const OGRSpatialReference *GDALAVIFDataset::GetGCPSpatialRef() const
{
    return this->GetSpatialRef();
}
#endif

/************************************************************************/
/*                              Init()                                  */
/************************************************************************/

bool GDALAVIFDataset::Init(GDALOpenInfo *poOpenInfo)
{
    m_decoder = avifDecoderCreate();
    if (!m_decoder)
        return false;

    std::string osFilename(poOpenInfo->pszFilename);
    VSIVirtualHandleUniquePtr fp(poOpenInfo->fpL);
    poOpenInfo->fpL = nullptr;

    if (STARTS_WITH_CI(poOpenInfo->pszFilename, "AVIF:"))
    {
        const char *pszPartPos = poOpenInfo->pszFilename + strlen("AVIF:");
        const char *pszNextColumn = strchr(pszPartPos, ':');
        if (pszNextColumn == nullptr)
            return false;
        m_iPart = atoi(pszPartPos);
        if (m_iPart <= 0)
            return false;
        osFilename = pszNextColumn + 1;
        fp.reset(VSIFOpenL(osFilename.c_str(), "rb"));
        if (fp == nullptr)
            return false;
    }

    auto gdalIO = std::make_unique<GDALAVIFIO>(std::move(fp));
    avifDecoderSetIO(m_decoder, reinterpret_cast<avifIO *>(gdalIO.release()));

    const auto avifErr = avifDecoderParse(m_decoder);
    if (avifErr != AVIF_RESULT_OK)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "avifDecoderParse() failed with: %s",
                 avifResultToString(avifErr));
        return false;
    }

    // AVIF limit is 65,536 x 65,536 pixels;
    nRasterXSize = static_cast<int>(m_decoder->image->width);
    nRasterYSize = static_cast<int>(m_decoder->image->height);

    if (m_decoder->image->depth > 12)
    {
        CPLError(CE_Failure, CPLE_NotSupported, "Unsupported AVIF depth: %u",
                 m_decoder->image->depth);
        return false;
    }

    const auto eDataType =
        (m_decoder->image->depth <= 8) ? GDT_Byte : GDT_UInt16;
    const int l_nBands = m_decoder->image->yuvFormat == AVIF_PIXEL_FORMAT_YUV400
                             ? (m_decoder->alphaPresent ? 2 : 1)
                         : m_decoder->alphaPresent ? 4
                                                   : 3;

    if (m_decoder->image->yuvFormat == AVIF_PIXEL_FORMAT_YUV444)
        GDALDataset::SetMetadataItem("YUV_SUBSAMPLING", "444",
                                     "IMAGE_STRUCTURE");
    else if (m_decoder->image->yuvFormat == AVIF_PIXEL_FORMAT_YUV422)
        GDALDataset::SetMetadataItem("YUV_SUBSAMPLING", "422",
                                     "IMAGE_STRUCTURE");
    else if (m_decoder->image->yuvFormat == AVIF_PIXEL_FORMAT_YUV420)
        GDALDataset::SetMetadataItem("YUV_SUBSAMPLING", "420",
                                     "IMAGE_STRUCTURE");

    for (int i = 0; i < l_nBands; ++i)
    {
        SetBand(i + 1, new GDALAVIFRasterBand(
                           this, i + 1, eDataType,
                           static_cast<int>(m_decoder->image->depth)));
    }

    if (m_iPart == 0)
    {
        if (m_decoder->imageCount > 1)
        {
            CPLStringList aosSubDS;
            for (int i = 0; i < m_decoder->imageCount; i++)
            {
                aosSubDS.SetNameValue(
                    CPLSPrintf("SUBDATASET_%d_NAME", i + 1),
                    CPLSPrintf("AVIF:%d:%s", i + 1, poOpenInfo->pszFilename));
                aosSubDS.SetNameValue(CPLSPrintf("SUBDATASET_%d_DESC", i + 1),
                                      CPLSPrintf("Subdataset %d", i + 1));
            }
            GDALDataset::SetMetadata(aosSubDS.List(), "SUBDATASETS");
        }
    }
    else if (m_iPart > m_decoder->imageCount)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Invalid image part number. Maximum allowed is %d",
                 m_decoder->imageCount);
        return false;
    }
    else
    {
        m_iPart--;
    }

    if (m_decoder->image->exif.size >= 8)
    {
        VSILFILE *fpEXIF =
            VSIFileFromMemBuffer(nullptr, m_decoder->image->exif.data,
                                 m_decoder->image->exif.size, false);
        int nExifOffset = 0;
        int nInterOffset = 0;
        int nGPSOffset = 0;
        char **papszEXIFMetadata = nullptr;
#ifdef CPL_LSB
        const bool bSwab = m_decoder->image->exif.data[0] == 0x4d;
#else
        const bool bSwab = m_decoder->image->exif.data[0] == 0x49;
#endif
        constexpr int nTIFFHEADER = 0;
        uint32_t nTiffDirStart;
        memcpy(&nTiffDirStart, m_decoder->image->exif.data + 4,
               sizeof(uint32_t));
        if (bSwab)
        {
            CPL_LSBPTR32(&nTiffDirStart);
        }
        EXIFExtractMetadata(papszEXIFMetadata, fpEXIF, nTiffDirStart, bSwab,
                            nTIFFHEADER, nExifOffset, nInterOffset, nGPSOffset);

        if (nExifOffset > 0)
        {
            EXIFExtractMetadata(papszEXIFMetadata, fpEXIF, nExifOffset, bSwab,
                                nTIFFHEADER, nExifOffset, nInterOffset,
                                nGPSOffset);
        }
        if (nInterOffset > 0)
        {
            EXIFExtractMetadata(papszEXIFMetadata, fpEXIF, nInterOffset, bSwab,
                                nTIFFHEADER, nExifOffset, nInterOffset,
                                nGPSOffset);
        }
        if (nGPSOffset > 0)
        {
            EXIFExtractMetadata(papszEXIFMetadata, fpEXIF, nGPSOffset, bSwab,
                                nTIFFHEADER, nExifOffset, nInterOffset,
                                nGPSOffset);
        }
        VSIFCloseL(fpEXIF);
        GDALDataset::SetMetadata(papszEXIFMetadata, "EXIF");
        CSLDestroy(papszEXIFMetadata);
    }

    if (m_decoder->image->xmp.size > 0)
    {
        const std::string osXMP(
            reinterpret_cast<const char *>(m_decoder->image->xmp.data),
            m_decoder->image->xmp.size);
        const char *const apszMD[] = {osXMP.c_str(), nullptr};
        GDALDataset::SetMetadata(const_cast<char **>(apszMD), "xml:XMP");
    }

    if (m_decoder->image->icc.size > 0)
    {
        // Escape the profile.
        char *pszBase64Profile =
            CPLBase64Encode(static_cast<int>(m_decoder->image->icc.size),
                            m_decoder->image->icc.data);

        // Set ICC profile metadata.
        SetMetadataItem("SOURCE_ICC_PROFILE", pszBase64Profile,
                        "COLOR_PROFILE");

        CPLFree(pszBase64Profile);
    }

    // Initialize any PAM information.
    if (m_decoder->imageCount > 1)
    {
        SetSubdatasetName(CPLSPrintf("%d", m_iPart + 1));
        SetPhysicalFilename(osFilename.c_str());
    }
    SetDescription(poOpenInfo->pszFilename);
    TryLoadXML(poOpenInfo->GetSiblingFiles());

    return true;
}

/************************************************************************/
/*                          OpenStaticPAM()                             */
/************************************************************************/

/* static */
GDALPamDataset *GDALAVIFDataset::OpenStaticPAM(GDALOpenInfo *poOpenInfo)
{
    if (!AVIFDriverIdentify(poOpenInfo))
        return nullptr;

    if (poOpenInfo->eAccess == GA_Update)
    {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "Update of existing AVIF file not supported");
        return nullptr;
    }

    auto poDS = std::make_unique<GDALAVIFDataset>();
    if (!poDS->Init(poOpenInfo))
        return nullptr;

    return poDS.release();
}

/************************************************************************/
/*                             CreateCopy()                             */
/************************************************************************/

/* static */
GDALDataset *GDALAVIFDataset::CreateCopy(const char *pszFilename,
                                         GDALDataset *poSrcDS,
                                         int /* bStrict */, char **papszOptions,
                                         GDALProgressFunc pfnProgress,
                                         void *pProgressData)
{
    auto poDrv = GetGDALDriverManager()->GetDriverByName(DRIVER_NAME);
    if (poDrv && poDrv->GetMetadataItem(GDAL_DMD_CREATIONOPTIONLIST) == nullptr)
    {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "This build of libavif has been done without any AV1 encoder");
        return nullptr;
    }

    // Perform various validations on source dataset
    const int nXSize = poSrcDS->GetRasterXSize();
    const int nYSize = poSrcDS->GetRasterYSize();
    const int nBands = poSrcDS->GetRasterCount();

    if (nXSize > 65536 || nYSize > 65536)
    {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "Too big source dataset. Maximum AVIF image dimension is "
                 "65,536 x 65,536 pixels");
        return nullptr;
    }
    if (nBands != 1 && nBands != 2 && nBands != 3 && nBands != 4)
    {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "Unsupported number of bands: only 1 (Gray), 2 (Gray+Alpha) "
                 "3 (RGB) or 4 (RGBA) bands are supported");
        return nullptr;
    }

    const auto poFirstBand = poSrcDS->GetRasterBand(1);
    if (poFirstBand->GetColorTable())
    {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "Source dataset with color table unsupported. Use "
                 "gdal_translate -expand rgb|rgba first");
        return nullptr;
    }

    const auto eDT = poFirstBand->GetRasterDataType();
    if (eDT != GDT_Byte && eDT != GDT_UInt16)
    {
        CPLError(
            CE_Failure, CPLE_NotSupported,
            "Unsupported data type: only Byte or UInt16 bands are supported");
        return nullptr;
    }

    int nBits = eDT == GDT_Byte ? 8 : 12;
    const char *pszNBITS = CSLFetchNameValue(papszOptions, "NBITS");
    if (pszNBITS)
    {
        nBits = atoi(pszNBITS);
    }
    else if (eDT == GDT_UInt16)
    {
        pszNBITS = poFirstBand->GetMetadataItem("NBITS", "IMAGE_STRUCTURE");
        if (pszNBITS)
        {
            nBits = atoi(pszNBITS);
        }
    }
    if ((eDT == GDT_Byte && nBits != 8) ||
        (eDT == GDT_UInt16 && nBits != 10 && nBits != 12))
    {
        CPLError(CE_Failure, CPLE_FileIO,
                 "Invalid/inconsistent bit depth w.r.t data type");
        return nullptr;
    }

    const int nQuality =
        std::clamp(atoi(CSLFetchNameValueDef(papszOptions, "QUALITY",
                                             DEFAULT_QUALITY_STR)),
                   0, 100);
    const int nQualityAlpha =
        std::clamp(atoi(CSLFetchNameValueDef(papszOptions, "QUALITY_ALPHA",
                                             DEFAULT_QUALITY_ALPHA_STR)),
                   0, 100);

    // Create AVIF image.
    avifPixelFormat ePixelFormat =
        nBands <= 2 ? AVIF_PIXEL_FORMAT_YUV400 : AVIF_PIXEL_FORMAT_YUV444;
    if (nBands >= 3)
    {
        const char *pszYUV_SUBSAMPLING =
            CSLFetchNameValueDef(papszOptions, "YUV_SUBSAMPLING", "444");
        if (EQUAL(pszYUV_SUBSAMPLING, "422"))
            ePixelFormat = AVIF_PIXEL_FORMAT_YUV422;
        else if (EQUAL(pszYUV_SUBSAMPLING, "420"))
            ePixelFormat = AVIF_PIXEL_FORMAT_YUV420;

        if (nQuality == 100 && nQualityAlpha == 100 &&
            ePixelFormat != AVIF_PIXEL_FORMAT_YUV444)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Only YUV_SUBSAMPLING=444 is supported for lossless "
                     "encoding");
            return nullptr;
        }
    }

    // Create empty output file
    VSIVirtualHandleUniquePtr fp(VSIFOpenL(pszFilename, "wb"));
    if (!fp)
    {
        CPLError(CE_Failure, CPLE_FileIO, "Cannot create file %s", pszFilename);
        return nullptr;
    }

    avifImage *image = avifImageCreate(nXSize, nYSize, nBits, ePixelFormat);
    if (!image)
    {
        return nullptr;
    }

    avifRGBImage rgb;
    memset(&rgb, 0, sizeof(rgb));
    avifRGBImageSetDefaults(&rgb, image);

    rgb.format =
        nBands == 1 || nBands == 3 ? AVIF_RGB_FORMAT_RGB : AVIF_RGB_FORMAT_RGBA;

    avifResult avifErr;

#if AVIF_VERSION_MAJOR >= 1
    avifErr = avifRGBImageAllocatePixels(&rgb);
    if (avifErr != AVIF_RESULT_OK)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "avifRGBImageAllocatePixels() failed with: %s",
                 avifResultToString(avifErr));
        avifImageDestroy(image);
        return nullptr;
    }
#else
    avifRGBImageAllocatePixels(&rgb);
    if (!rgb.pixels)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "avifRGBImageAllocatePixels() failed");
        avifImageDestroy(image);
        return nullptr;
    }
#endif

    const int nDTSize = GDALGetDataTypeSizeBytes(eDT);
    GDALRasterIOExtraArg sExtraArg;
    INIT_RASTERIO_EXTRA_ARG(sExtraArg);

    CPLErr eErr;
    if (nBands == 1)
    {
        int anBands[] = {1, 1, 1};
        eErr = poSrcDS->RasterIO(GF_Read, 0, 0, nXSize, nYSize, rgb.pixels,
                                 nXSize, nYSize, eDT, 3, anBands, nDTSize * 3,
                                 static_cast<int>(rgb.rowBytes), nDTSize,
                                 &sExtraArg);
    }
    else if (nBands == 2)
    {
        int anBands[] = {1, 1, 1, 2};
        eErr = poSrcDS->RasterIO(GF_Read, 0, 0, nXSize, nYSize, rgb.pixels,
                                 nXSize, nYSize, eDT, 4, anBands, nDTSize * 4,
                                 static_cast<int>(rgb.rowBytes), nDTSize,
                                 &sExtraArg);
    }
    else
    {
        eErr = poSrcDS->RasterIO(
            GF_Read, 0, 0, nXSize, nYSize, rgb.pixels, nXSize, nYSize, eDT,
            nBands, nullptr, nDTSize * nBands, static_cast<int>(rgb.rowBytes),
            nDTSize, &sExtraArg);
    }
    if (eErr != CE_None)
    {
        avifImageDestroy(image);
        avifRGBImageFreePixels(&rgb);
        return nullptr;
    }

    if (nQuality == 100 && nQualityAlpha == 100)
    {
        // Cf https://github.com/AOMediaCodec/libavif/blob/0d3e5e215dffbbd6afbf917ce00c84de599ba410/apps/avifenc.c#L1952
        image->matrixCoefficients = AVIF_MATRIX_COEFFICIENTS_IDENTITY;
    }
    else
    {
        // Cf https://github.com/AOMediaCodec/libavif/blob/0d3e5e215dffbbd6afbf917ce00c84de599ba410/apps/avifenc.c#L1434
        image->matrixCoefficients = AVIF_MATRIX_COEFFICIENTS_BT601;
    }

    // Cf https://github.com/AOMediaCodec/libavif/blob/0d3e5e215dffbbd6afbf917ce00c84de599ba410/apps/avifenc.c#L2249
    // The final image has no ICC profile, the user didn't specify any CICP, and the source
    // image didn't provide any CICP. Explicitly signal SRGB CP/TC here, as 2/2/x will be
    // interpreted as SRGB anyway.
    image->colorPrimaries = AVIF_COLOR_PRIMARIES_BT709;
    image->transferCharacteristics = AVIF_TRANSFER_CHARACTERISTICS_SRGB;

    image->yuvRange = AVIF_RANGE_FULL;
    image->alphaPremultiplied = 0;

    avifErr = avifImageRGBToYUV(image, &rgb);
    if (avifErr != AVIF_RESULT_OK)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "avifImageRGBToYUV() failed with: %s",
                 avifResultToString(avifErr));
        avifImageDestroy(image);
        avifRGBImageFreePixels(&rgb);
        return nullptr;
    }

    avifEncoder *encoder = avifEncoderCreate();
    if (!encoder)
    {
        avifImageDestroy(image);
        avifRGBImageFreePixels(&rgb);
        return nullptr;
    }

    const char *pszCodec = CSLFetchNameValueDef(papszOptions, "CODEC", "AUTO");
    if (!EQUAL(pszCodec, "AUTO"))
    {
        encoder->codecChoice =
            avifCodecChoiceFromName(CPLString(pszCodec).tolower().c_str());
    }

    const char *pszThreads = CSLFetchNameValueDef(
        papszOptions, "NUM_THREADS",
        CPLGetConfigOption("GDAL_NUM_THREADS", "ALL_CPUS"));
    if (pszThreads && !EQUAL(pszThreads, "ALL_CPUS"))
        encoder->maxThreads = atoi(pszThreads);
    else
        encoder->maxThreads = CPLGetNumCPUs();

#if AVIF_VERSION_MAJOR >= 1
    encoder->quality = nQuality;
    encoder->qualityAlpha = nQualityAlpha;
#else
    // Cf https://github.com/AOMediaCodec/libavif/blob/0d3e5e215dffbbd6afbf917ce00c84de599ba410/src/write.c#L1119
    const int nQuantizer = ((100 - nQuality) * 63 + 50) / 100;
    encoder->minQuantizer = nQuantizer;
    encoder->maxQuantizer = nQuantizer;
    const int nQuantizerAlpha = ((100 - nQualityAlpha) * 63 + 50) / 100;
    encoder->minQuantizerAlpha = nQuantizerAlpha;
    encoder->maxQuantizerAlpha = nQuantizerAlpha;
#endif

    encoder->speed = std::clamp(
        atoi(CSLFetchNameValueDef(papszOptions, "SPEED", DEFAULT_SPEED_STR)), 0,
        10);

    if (CPLTestBool(
            CSLFetchNameValueDef(papszOptions, "WRITE_EXIF_METADATA", "YES")))
    {
        char **papszEXIFMD = poSrcDS->GetMetadata("EXIF");
        if (papszEXIFMD)
        {
            GUInt32 nDataSize = 0;
            GByte *pabyEXIF =
                EXIFCreate(papszEXIFMD, nullptr, 0, 0, 0, &nDataSize);
            if (pabyEXIF)
            {
                CPLAssert(nDataSize > 6 &&
                          memcmp(pabyEXIF, "Exif\0\0", 6) == 0);

#if AVIF_VERSION_MAJOR >= 1
                CPL_IGNORE_RET_VAL(avifImageSetMetadataExif(image, pabyEXIF + 6,
                                                            nDataSize - 6));
#else
                avifImageSetMetadataExif(image, pabyEXIF + 6, nDataSize - 6);
#endif
                CPLFree(pabyEXIF);
            }
        }
    }

    if (CPLTestBool(CSLFetchNameValueDef(papszOptions, "WRITE_XMP", "YES")))
    {
        CSLConstList papszXMP = poSrcDS->GetMetadata("xml:XMP");
        if (papszXMP && papszXMP[0])
        {
#if AVIF_VERSION_MAJOR >= 1
            CPL_IGNORE_RET_VAL(avifImageSetMetadataXMP(
                image, reinterpret_cast<const uint8_t *>(papszXMP[0]),
                strlen(papszXMP[0])));
#else
            avifImageSetMetadataXMP(
                image, reinterpret_cast<const uint8_t *>(papszXMP[0]),
                strlen(papszXMP[0]));
#endif
        }
    }

#if AVIF_VERSION_MAJOR >= 1
    const char *pszICCProfile =
        CSLFetchNameValue(papszOptions, "SOURCE_ICC_PROFILE");
    if (pszICCProfile == nullptr)
    {
        pszICCProfile =
            poSrcDS->GetMetadataItem("SOURCE_ICC_PROFILE", "COLOR_PROFILE");
    }
    if (pszICCProfile && pszICCProfile[0] != '\0')
    {
        char *pEmbedBuffer = CPLStrdup(pszICCProfile);
        const GInt32 nEmbedLen =
            CPLBase64DecodeInPlace(reinterpret_cast<GByte *>(pEmbedBuffer));
        CPL_IGNORE_RET_VAL(avifImageSetProfileICC(
            image, reinterpret_cast<const uint8_t *>(pEmbedBuffer), nEmbedLen));
        CPLFree(pEmbedBuffer);
    }
#endif

    avifErr =
        avifEncoderAddImage(encoder, image, 1, AVIF_ADD_IMAGE_FLAG_SINGLE);
    if (avifErr != AVIF_RESULT_OK)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "avifEncoderAddImage() failed with: %s",
                 avifResultToString(avifErr));
        avifImageDestroy(image);
        avifEncoderDestroy(encoder);
        avifRGBImageFreePixels(&rgb);
        return nullptr;
    }

    avifRWData avifOutput = AVIF_DATA_EMPTY;
    avifErr = avifEncoderFinish(encoder, &avifOutput);

    avifEncoderDestroy(encoder);
    avifImageDestroy(image);
    avifRGBImageFreePixels(&rgb);

    if (avifErr != AVIF_RESULT_OK)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "avifEncoderFinish() failed with: %s",
                 avifResultToString(avifErr));
        return nullptr;
    }

    const size_t nSize = static_cast<size_t>(avifOutput.size);
    if (fp->Write(avifOutput.data, 1, nSize) != nSize || fp->Close() != 0)
    {
        CPLError(CE_Failure, CPLE_FileIO,
                 "Could not write %" PRIu64 " bytes into file %s",
                 static_cast<uint64_t>(nSize), pszFilename);
        avifRWDataFree(&avifOutput);
        return nullptr;
    }
    avifRWDataFree(&avifOutput);

    fp.reset();

    if (pfnProgress)
        pfnProgress(1.0, "", pProgressData);

    // Re-open file and clone missing info to PAM
    GDALOpenInfo oOpenInfo(pszFilename, GA_ReadOnly);
    auto poDS = OpenStaticPAM(&oOpenInfo);
    if (poDS)
    {
        // Do not create a .aux.xml file just for AREA_OR_POINT=Area
        const char *pszAreaOfPoint =
            poSrcDS->GetMetadataItem(GDALMD_AREA_OR_POINT);
        if (pszAreaOfPoint && EQUAL(pszAreaOfPoint, GDALMD_AOP_AREA))
        {
            poDS->SetMetadataItem(GDALMD_AREA_OR_POINT, GDALMD_AOP_AREA);
            poDS->SetPamFlags(poDS->GetPamFlags() & ~GPF_DIRTY);
        }

        int nPamMask = GCIF_PAM_DEFAULT;
        poDS->CloneInfo(poSrcDS, nPamMask);
    }

    return poDS;
}

/************************************************************************/
/*                         GDALAVIFDriver                               */
/************************************************************************/

class GDALAVIFDriver final : public GDALDriver
{
    bool m_bMetadataInitialized = false;
    void InitMetadata();

  public:
    const char *GetMetadataItem(const char *pszName,
                                const char *pszDomain = "") override
    {
        if (EQUAL(pszName, GDAL_DMD_CREATIONOPTIONLIST))
        {
            InitMetadata();
        }
        return GDALDriver::GetMetadataItem(pszName, pszDomain);
    }

    char **GetMetadata(const char *pszDomain) override
    {
        InitMetadata();
        return GDALDriver::GetMetadata(pszDomain);
    }
};

void GDALAVIFDriver::InitMetadata()
{
    if (m_bMetadataInitialized)
        return;
    m_bMetadataInitialized = true;

    std::vector<std::string> aosCodecNames;
    for (auto eMethod : {AVIF_CODEC_CHOICE_AUTO, AVIF_CODEC_CHOICE_AOM,
                         AVIF_CODEC_CHOICE_RAV1E, AVIF_CODEC_CHOICE_SVT})
    {
        const char *pszName =
            avifCodecName(eMethod, AVIF_CODEC_FLAG_CAN_ENCODE);
        if (pszName)
        {
            aosCodecNames.push_back(eMethod == AVIF_CODEC_CHOICE_AUTO
                                        ? CPLString("AUTO")
                                        : CPLString(pszName).toupper());
        }
    }

    if (aosCodecNames.empty())
        return;

    CPLXMLTreeCloser oTree(
        CPLCreateXMLNode(nullptr, CXT_Element, "CreationOptionList"));

    {
        auto psOption = CPLCreateXMLNode(oTree.get(), CXT_Element, "Option");
        CPLAddXMLAttributeAndValue(psOption, "name", "CODEC");
        CPLAddXMLAttributeAndValue(psOption, "type", "string-select");
        CPLAddXMLAttributeAndValue(psOption, "description",
                                   "Compression CODEC");
        CPLAddXMLAttributeAndValue(psOption, "default", "AUTO");
        for (const std::string &osCodecName : aosCodecNames)
        {
            auto poValueNode = CPLCreateXMLNode(psOption, CXT_Element, "Value");
            CPLCreateXMLNode(poValueNode, CXT_Text, osCodecName.c_str());
        }
    }

    {
        auto psOption = CPLCreateXMLNode(oTree.get(), CXT_Element, "Option");
        CPLAddXMLAttributeAndValue(psOption, "name", "QUALITY");
        CPLAddXMLAttributeAndValue(psOption, "type", "int");
        CPLAddXMLAttributeAndValue(
            psOption, "description",
            "Quality for non-alpha channels (0=worst, 100=best/lossless)");
        CPLAddXMLAttributeAndValue(psOption, "default", DEFAULT_QUALITY_STR);
        CPLAddXMLAttributeAndValue(psOption, "min", "0");
        CPLAddXMLAttributeAndValue(psOption, "max", "100");
    }

    {
        auto psOption = CPLCreateXMLNode(oTree.get(), CXT_Element, "Option");
        CPLAddXMLAttributeAndValue(psOption, "name", "QUALITY_ALPHA");
        CPLAddXMLAttributeAndValue(psOption, "type", "int");
        CPLAddXMLAttributeAndValue(
            psOption, "description",
            "Quality for alpha channel (0=worst, 100=best/lossless)");
        CPLAddXMLAttributeAndValue(psOption, "default",
                                   DEFAULT_QUALITY_ALPHA_STR);
        CPLAddXMLAttributeAndValue(psOption, "min", "0");
        CPLAddXMLAttributeAndValue(psOption, "max", "100");
    }

    {
        auto psOption = CPLCreateXMLNode(oTree.get(), CXT_Element, "Option");
        CPLAddXMLAttributeAndValue(psOption, "name", "SPEED");
        CPLAddXMLAttributeAndValue(psOption, "type", "int");
        CPLAddXMLAttributeAndValue(psOption, "description",
                                   "Encoder speed (0=slowest, 10=fastest)");
        CPLAddXMLAttributeAndValue(psOption, "default", DEFAULT_SPEED_STR);
        CPLAddXMLAttributeAndValue(psOption, "min", "0");
        CPLAddXMLAttributeAndValue(psOption, "max", "10");
    }

    {
        auto psOption = CPLCreateXMLNode(oTree.get(), CXT_Element, "Option");
        CPLAddXMLAttributeAndValue(psOption, "name", "NUM_THREADS");
        CPLAddXMLAttributeAndValue(psOption, "type", "string");
        CPLAddXMLAttributeAndValue(
            psOption, "description",
            "Number of worker threads for compression. Can be set to ALL_CPUS");
        CPLAddXMLAttributeAndValue(psOption, "default", "ALL_CPUS");
    }

    {
        auto psOption = CPLCreateXMLNode(oTree.get(), CXT_Element, "Option");
        CPLAddXMLAttributeAndValue(psOption, "name", "WRITE_EXIF_METADATA");
        CPLAddXMLAttributeAndValue(psOption, "type", "boolean");
        CPLAddXMLAttributeAndValue(psOption, "description",
                                   "Whether to write EXIF metadata");
        CPLAddXMLAttributeAndValue(psOption, "default", "YES");
    }

    {
        auto psOption = CPLCreateXMLNode(oTree.get(), CXT_Element, "Option");
        CPLAddXMLAttributeAndValue(psOption, "name", "WRITE_XMP");
        CPLAddXMLAttributeAndValue(psOption, "type", "boolean");
        CPLAddXMLAttributeAndValue(psOption, "description",
                                   "Whether to write XMP metadata");
        CPLAddXMLAttributeAndValue(psOption, "default", "YES");
    }

#if AVIF_VERSION_MAJOR >= 1
    {
        auto psOption = CPLCreateXMLNode(oTree.get(), CXT_Element, "Option");
        CPLAddXMLAttributeAndValue(psOption, "name", "SOURCE_ICC_PROFILE");
        CPLAddXMLAttributeAndValue(psOption, "type", "string");
        CPLAddXMLAttributeAndValue(psOption, "description",
                                   "ICC profile encoded in Base64");
    }
#endif

    {
        auto psOption = CPLCreateXMLNode(oTree.get(), CXT_Element, "Option");
        CPLAddXMLAttributeAndValue(psOption, "name", "NBITS");
        CPLAddXMLAttributeAndValue(psOption, "type", "int");
        CPLAddXMLAttributeAndValue(psOption, "description",
                                   "Bit depth. Valid values are 8, 10, 12.");
    }

    {
        auto psOption = CPLCreateXMLNode(oTree.get(), CXT_Element, "Option");
        CPLAddXMLAttributeAndValue(psOption, "name", "YUV_SUBSAMPLING");
        CPLAddXMLAttributeAndValue(psOption, "type", "string-select");
        CPLAddXMLAttributeAndValue(
            psOption, "description",
            "Subsampling factor for YUV colorspace (for RGB or RGBA)");
        CPLAddXMLAttributeAndValue(psOption, "default", "444");

        for (const char *pszValue : {"444", "422", "420"})
        {
            auto poValueNode = CPLCreateXMLNode(psOption, CXT_Element, "Value");
            CPLCreateXMLNode(poValueNode, CXT_Text, pszValue);
        }
    }

    char *pszXML = CPLSerializeXMLTree(oTree.get());
    GDALDriver::SetMetadataItem(GDAL_DMD_CREATIONOPTIONLIST, pszXML);
    CPLFree(pszXML);
}

/************************************************************************/
/*                       GDALRegister_AVIF()                            */
/************************************************************************/

void GDALRegister_AVIF()

{
    if (!GDAL_CHECK_VERSION("AVIF driver"))
        return;

    if (GDALGetDriverByName(DRIVER_NAME) != nullptr)
        return;

    // Check libavif runtime vs compile-time versions
    const char *pszVersion = avifVersion();
    const CPLStringList aosVersionTokens(
        CSLTokenizeString2(pszVersion, ".", 0));
    if (aosVersionTokens.size() >= 2 &&
        std::string(aosVersionTokens[0])
                .append(".")
                .append(aosVersionTokens[1]) !=
            CPLSPrintf("%d.%d", AVIF_VERSION_MAJOR, AVIF_VERSION_MINOR))
    {
        const std::string osExpectedVersion(
            CPLSPrintf("%d.%d.%d", AVIF_VERSION_MAJOR, AVIF_VERSION_MINOR,
                       AVIF_VERSION_PATCH));
        CPLError(CE_Warning, CPLE_AppDefined,
                 "GDAL AVIF driver was built against libavif %s but is running "
                 "against %s. Runtime issues could occur",
                 osExpectedVersion.c_str(), avifVersion());
    }

    auto poDriver = std::make_unique<GDALAVIFDriver>();
    auto poDM = GetGDALDriverManager();
    bool bMayHaveWriteSupport = true;
    if (!poDM->IsKnownDriver("AVIF"))
    {
        // If we are not built as a defered plugin, check now if libavif has
        // write support
        bMayHaveWriteSupport =
            poDriver->GetMetadataItem(GDAL_DMD_CREATIONOPTIONLIST) != nullptr;
    }

    AVIFDriverSetCommonMetadata(poDriver.get(), bMayHaveWriteSupport);

    poDriver->pfnOpen = GDALAVIFDataset::Open;
    if (bMayHaveWriteSupport)
        poDriver->pfnCreateCopy = GDALAVIFDataset::CreateCopy;

    poDM->RegisterDriver(poDriver.release());
}
