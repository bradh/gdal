/******************************************************************************
 *
 * Project:  HEIF read-only Driver
 * Author:   Even Rouault <even.rouault at spatialys.com>
 *
 ******************************************************************************
 * Copyright (c) 2020, Even Rouault <even.rouault at spatialys.com>
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
#include "ogr_spatialref.h"

#include "libheif/heif.h"
#include "st0601.h"

#include <cpl_port.h>
#include <cstddef>
#include <vector>

extern "C" void CPL_DLL GDALRegister_HEIF();

// g++ -fPIC -std=c++11 frmts/heif/heifdataset.cpp -Iport -Igcore -Iogr
// -Iogr/ogrsf_frmts -I$HOME/heif/install-ubuntu-18.04/include
// -L$HOME/heif/install-ubuntu-18.04/lib -lheif -shared -o gdal_HEIF.so -L.
// -lgdal

#define BUILD_LIBHEIF_VERSION(x, y, z)                                         \
    (((x) << 24) | ((y) << 16) | ((z) << 8) | 0)

#if LIBHEIF_NUMERIC_VERSION >= BUILD_LIBHEIF_VERSION(1, 3, 0)
#define HAS_CUSTOM_FILE_READER
#endif

/************************************************************************/
/*                        GDALHEIFDataset                               */
/************************************************************************/

class GDALHEIFDataset final : public GDALPamDataset
{
    friend class GDALHEIFRasterBand;

    heif_context *m_hCtxt = nullptr;
    heif_image_handle *m_hImageHandle = nullptr;
    heif_image *m_hImage = nullptr;
    bool m_bFailureDecoding = false;
    std::vector<std::unique_ptr<GDALHEIFDataset>> m_apoOvrDS{};
    bool m_bIsThumbnail = false;

#ifdef HAS_CUSTOM_FILE_READER
    heif_reader m_oReader{};
    VSILFILE *m_fpL = nullptr;
    vsi_l_offset m_nSize = 0;

    static int64_t GetPositionCbk(void *userdata);
    static int ReadCbk(void *data, size_t size, void *userdata);
    static int SeekCbk(int64_t position, void *userdata);
    static enum heif_reader_grow_status WaitForFileSizeCbk(int64_t target_size,
                                                           void *userdata);
#endif

    bool Init(GDALOpenInfo *poOpenInfo);
    void ReadMetadata();
    void OpenThumbnails();

    double adfGeoTransform[6];
    int bGeoTransformSet;
    int nGCPCount;
    OGRSpatialReference m_oGCPSRS{};
    GDAL_GCP *pasGCPList;

  public:
    GDALHEIFDataset();
    ~GDALHEIFDataset();

    static int Identify(GDALOpenInfo *poOpenInfo);
    static GDALDataset *Open(GDALOpenInfo *poOpenInfo);

    int GetGCPCount() override;
    const OGRSpatialReference *GetGCPSpatialRef() const override;
    const GDAL_GCP *GetGCPs() override;

    CPLErr GetGeoTransform(double *padfTransform) override;
    const OGRSpatialReference *GetSpatialRef() const override;
};

/************************************************************************/
/*                       GDALHEIFRasterBand                             */
/************************************************************************/

class GDALHEIFRasterBand final : public GDALPamRasterBand
{
  protected:
    CPLErr IReadBlock(int, int, void *) override;

  public:
    GDALHEIFRasterBand(GDALHEIFDataset *poDSIn, int nBandIn);

    GDALColorInterp GetColorInterpretation() override
    {
        return static_cast<GDALColorInterp>(GCI_RedBand + nBand - 1);
    }

    int GetOverviewCount() override
    {
        GDALHEIFDataset *poGDS = static_cast<GDALHEIFDataset *>(poDS);
        return static_cast<int>(poGDS->m_apoOvrDS.size());
    }

    GDALRasterBand *GetOverview(int idx) override
    {
        if (idx < 0 || idx >= GetOverviewCount())
            return nullptr;
        GDALHEIFDataset *poGDS = static_cast<GDALHEIFDataset *>(poDS);
        return poGDS->m_apoOvrDS[idx]->GetRasterBand(nBand);
    }
};

/************************************************************************/
/*                         GDALHEIFDataset()                            */
/************************************************************************/

GDALHEIFDataset::GDALHEIFDataset()
    : m_hCtxt(heif_context_alloc()), bGeoTransformSet(FALSE),
    nGCPCount(0), pasGCPList(nullptr)
    

{
#ifdef HAS_CUSTOM_FILE_READER
    m_oReader.reader_api_version = 1;
    m_oReader.get_position = GetPositionCbk;
    m_oReader.read = ReadCbk;
    m_oReader.seek = SeekCbk;
    m_oReader.wait_for_file_size = WaitForFileSizeCbk;
#endif
    m_oGCPSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    m_oGCPSRS.importFromWkt(SRS_WKT_WGS84_LAT_LONG);
}

/************************************************************************/
/*                         ~GDALHEIFDataset()                           */
/************************************************************************/

GDALHEIFDataset::~GDALHEIFDataset()
{
    if (m_hCtxt)
        heif_context_free(m_hCtxt);
#ifdef HAS_CUSTOM_FILE_READER
    if (m_fpL)
        VSIFCloseL(m_fpL);
#endif
    if (m_hImage)
        heif_image_release(m_hImage);
    if (m_hImageHandle)
        heif_image_handle_release(m_hImageHandle);
}

/************************************************************************/
/*                            Identify()                                */
/************************************************************************/

int GDALHEIFDataset::Identify(GDALOpenInfo *poOpenInfo)
{
    if (STARTS_WITH_CI(poOpenInfo->pszFilename, "HEIF:"))
        return true;

    if (poOpenInfo->nHeaderBytes < 12 || poOpenInfo->fpL == nullptr)
        return false;
#if LIBHEIF_NUMERIC_VERSION >= BUILD_LIBHEIF_VERSION(1, 4, 0)
    const auto res =
        heif_check_filetype(poOpenInfo->pabyHeader, poOpenInfo->nHeaderBytes);
    if (res == heif_filetype_yes_supported)
        return TRUE;
    if (res == heif_filetype_maybe)
        return -1;
    if (res == heif_filetype_yes_unsupported)
    {
        CPLDebug("HEIF", "HEIF file, but not supported by libheif");
    }
    return FALSE;
#else
    // Simplistic test...
    const unsigned char abySig1[] = "\x00"
                                    "\x00"
                                    "\x00"
                                    "\x20"
                                    "ftypheic";
    const unsigned char abySig2[] = "\x00"
                                    "\x00"
                                    "\x00"
                                    "\x18"
                                    "ftypheic";
    const unsigned char abySig3[] = "\x00"
                                    "\x00"
                                    "\x00"
                                    "\x18"
                                    "ftypmif1"
                                    "\x00"
                                    "\x00"
                                    "\x00"
                                    "\x00"
                                    "mif1heic";
    return (poOpenInfo->nHeaderBytes >= static_cast<int>(sizeof(abySig1)) &&
            memcmp(poOpenInfo->pabyHeader, abySig1, sizeof(abySig1)) == 0) ||
           (poOpenInfo->nHeaderBytes >= static_cast<int>(sizeof(abySig2)) &&
            memcmp(poOpenInfo->pabyHeader, abySig2, sizeof(abySig2)) == 0) ||
           (poOpenInfo->nHeaderBytes >= static_cast<int>(sizeof(abySig3)) &&
            memcmp(poOpenInfo->pabyHeader, abySig3, sizeof(abySig3)) == 0);
#endif
}

#ifdef HAS_CUSTOM_FILE_READER

/************************************************************************/
/*                          GetPositionCbk()                            */
/************************************************************************/

int64_t GDALHEIFDataset::GetPositionCbk(void *userdata)
{
    GDALHEIFDataset *poThis = static_cast<GDALHEIFDataset *>(userdata);
    return static_cast<int64_t>(VSIFTellL(poThis->m_fpL));
}

/************************************************************************/
/*                             ReadCbk()                                */
/************************************************************************/

int GDALHEIFDataset::ReadCbk(void *data, size_t size, void *userdata)
{
    GDALHEIFDataset *poThis = static_cast<GDALHEIFDataset *>(userdata);
    return VSIFReadL(data, size, 1, poThis->m_fpL) == 1 ? 0 : -1;
}

/************************************************************************/
/*                             SeekCbk()                                */
/************************************************************************/

int GDALHEIFDataset::SeekCbk(int64_t position, void *userdata)
{
    GDALHEIFDataset *poThis = static_cast<GDALHEIFDataset *>(userdata);
    return VSIFSeekL(poThis->m_fpL, static_cast<vsi_l_offset>(position),
                     SEEK_SET);
}

/************************************************************************/
/*                         WaitForFileSizeCbk()                         */
/************************************************************************/

enum heif_reader_grow_status
GDALHEIFDataset::WaitForFileSizeCbk(int64_t target_size, void *userdata)
{
    GDALHEIFDataset *poThis = static_cast<GDALHEIFDataset *>(userdata);
    if (target_size > static_cast<int64_t>(poThis->m_nSize))
        return heif_reader_grow_status_size_beyond_eof;
    return heif_reader_grow_status_size_reached;
}

#endif

/************************************************************************/
/*                              Init()                                  */
/************************************************************************/

bool GDALHEIFDataset::Init(GDALOpenInfo *poOpenInfo)
{
    CPLString osFilename(poOpenInfo->pszFilename);
#ifdef HAS_CUSTOM_FILE_READER
    VSILFILE *fpL;
#endif
    int iPart = 0;
    if (STARTS_WITH_CI(poOpenInfo->pszFilename, "HEIF:"))
    {
        const char *pszPartPos = poOpenInfo->pszFilename + strlen("HEIF:");
        const char *pszNextColumn = strchr(pszPartPos, ':');
        if (pszNextColumn == nullptr)
            return false;
        iPart = atoi(pszPartPos);
        if (iPart <= 0)
            return false;
        osFilename = pszNextColumn + 1;
#ifdef HAS_CUSTOM_FILE_READER
        fpL = VSIFOpenL(osFilename, "rb");
        if (fpL == nullptr)
            return false;
#endif
    }
    else
    {
#ifdef HAS_CUSTOM_FILE_READER
        fpL = poOpenInfo->fpL;
        poOpenInfo->fpL = nullptr;
#endif
    }

#ifdef HAS_CUSTOM_FILE_READER
    m_oReader.reader_api_version = 1;
    m_oReader.get_position = GetPositionCbk;
    m_oReader.read = ReadCbk;
    m_oReader.seek = SeekCbk;
    m_oReader.wait_for_file_size = WaitForFileSizeCbk;
    m_fpL = fpL;

    VSIFSeekL(m_fpL, 0, SEEK_END);
    m_nSize = VSIFTellL(m_fpL);
    VSIFSeekL(m_fpL, 0, SEEK_SET);

    auto err =
        heif_context_read_from_reader(m_hCtxt, &m_oReader, this, nullptr);
    if (err.code != heif_error_Ok)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "%s",
                 err.message ? err.message : "Cannot open file");
        return false;
    }
#else
    auto err =
        heif_context_read_from_file(m_hCtxt, osFilename.c_str(), nullptr);
    if (err.code != heif_error_Ok)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "%s",
                 err.message ? err.message : "Cannot open file");
        return false;
    }
#endif

    const int nSubdatasets =
        heif_context_get_number_of_top_level_images(m_hCtxt);
    if (iPart == 0)
    {
        if (nSubdatasets > 1)
        {
            CPLStringList aosSubDS;
            for (int i = 0; i < nSubdatasets; i++)
            {
                aosSubDS.SetNameValue(
                    CPLSPrintf("SUBDATASET_%d_NAME", i + 1),
                    CPLSPrintf("HEIF:%d:%s", i + 1, poOpenInfo->pszFilename));
                aosSubDS.SetNameValue(CPLSPrintf("SUBDATASET_%d_DESC", i + 1),
                                      CPLSPrintf("Subdataset %d", i + 1));
            }
            GDALDataset::SetMetadata(aosSubDS.List(), "SUBDATASETS");
        }
    }
    else if (iPart > nSubdatasets)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Invalid image part number. Maximum allowed is %d",
                 nSubdatasets);
        return false;
    }
    else
    {
        iPart--;
    }
    std::vector<heif_item_id> idArray(nSubdatasets);
    heif_context_get_list_of_top_level_image_IDs(m_hCtxt, &idArray[0],
                                                 nSubdatasets);
    const auto itemId = idArray[iPart];

    err = heif_context_get_image_handle(m_hCtxt, itemId, &m_hImageHandle);
    if (err.code != heif_error_Ok)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "%s",
                 err.message ? err.message : "Cannot open image");
        return false;
    }

    nRasterXSize = heif_image_handle_get_width(m_hImageHandle);
    nRasterYSize = heif_image_handle_get_height(m_hImageHandle);
    const int l_nBands =
        3 + (heif_image_handle_has_alpha_channel(m_hImageHandle) ? 1 : 0);
    for (int i = 0; i < l_nBands; i++)
    {
        SetBand(i + 1, new GDALHEIFRasterBand(this, i + 1));
    }

    ReadMetadata();

    OpenThumbnails();

    // Initialize any PAM information.
    SetDescription(poOpenInfo->pszFilename);
    TryLoadXML(poOpenInfo->GetSiblingFiles());

    return true;
}

/************************************************************************/
/*                         ReadMetadata()                               */
/************************************************************************/

void GDALHEIFDataset::ReadMetadata()
{
    const int nMDBlocks = heif_image_handle_get_number_of_metadata_blocks(
        m_hImageHandle, nullptr);
    if (nMDBlocks <= 0)
        return;

    std::vector<heif_item_id> idsMDBlock(nMDBlocks);
    heif_image_handle_get_list_of_metadata_block_IDs(m_hImageHandle, nullptr,
                                                     &idsMDBlock[0], nMDBlocks);
    for (const auto &id : idsMDBlock)
    {
        const char *pszType =
            heif_image_handle_get_metadata_type(m_hImageHandle, id);
        const size_t nCount =
            heif_image_handle_get_metadata_size(m_hImageHandle, id);
        if (pszType && EQUAL(pszType, "Exif") && nCount > 8 &&
            nCount < 1024 * 1024)
        {
            std::vector<GByte> data(nCount);
            heif_image_handle_get_metadata(m_hImageHandle, id, &data[0]);

            // There are 2 variants
            // - the one from
            // https://github.com/nokiatech/heif_conformance/blob/master/conformance_files/C034.heic
            //   where the TIFF file immediately starts
            // - the one found in iPhone files (among others), where there
            //   is first a 4-byte big-endian offset (after those initial 4
            //   bytes) that points to the TIFF file, with a "Exif\0\0" just
            //   before
            unsigned nTIFFFileOffset = 0;
            if (memcmp(&data[0], "II\x2a\x00", 4) == 0 ||
                memcmp(&data[0], "MM\x00\x2a", 4) == 0)
            {
                // do nothing
            }
            else
            {
                unsigned nOffset;
                memcpy(&nOffset, &data[0], 4);
                CPL_MSBPTR32(&nOffset);
                if (nOffset < nCount - 8 &&
                    (memcmp(&data[nOffset + 4], "II\x2a\x00", 4) == 0 ||
                     memcmp(&data[nOffset + 4], "MM\x00\x2a", 4) == 0))
                {
                    nTIFFFileOffset = nOffset + 4;
                }
                else
                {
                    continue;
                }
            }

            CPLString osTempFile;
            osTempFile.Printf("/vsimem/heif_exif_%p.tif", this);
            VSILFILE *fpTemp =
                VSIFileFromMemBuffer(osTempFile, &data[nTIFFFileOffset],
                                     nCount - nTIFFFileOffset, FALSE);
            char **papszMD = nullptr;

            const bool bLittleEndianTIFF = data[nTIFFFileOffset] == 'I' &&
                                           data[nTIFFFileOffset + 1] == 'I';
            const bool bLSBPlatform = CPL_IS_LSB != 0;
            const bool bSwabflag = bLittleEndianTIFF != bLSBPlatform;

            int nTIFFDirOff;
            memcpy(&nTIFFDirOff, &data[nTIFFFileOffset + 4], 4);
            if (bSwabflag)
            {
                CPL_SWAP32PTR(&nTIFFDirOff);
            }
            int nExifOffset = 0;
            int nInterOffset = 0;
            int nGPSOffset = 0;
            EXIFExtractMetadata(papszMD, fpTemp, nTIFFDirOff, bSwabflag, 0,
                                nExifOffset, nInterOffset, nGPSOffset);
            if (nExifOffset > 0)
            {
                EXIFExtractMetadata(papszMD, fpTemp, nExifOffset, bSwabflag, 0,
                                    nExifOffset, nInterOffset, nGPSOffset);
            }
            if (nGPSOffset > 0)
            {
                EXIFExtractMetadata(papszMD, fpTemp, nGPSOffset, bSwabflag, 0,
                                    nExifOffset, nInterOffset, nGPSOffset);
            }
            if (nInterOffset > 0)
            {
                EXIFExtractMetadata(papszMD, fpTemp, nInterOffset, bSwabflag, 0,
                                    nExifOffset, nInterOffset, nGPSOffset);
            }

            if (papszMD)
            {
                GDALDataset::SetMetadata(papszMD, "EXIF");
                CSLDestroy(papszMD);
            }

            VSIFCloseL(fpTemp);
            VSIUnlink(osTempFile);
        }
        else if (pszType && EQUAL(pszType, "mime"))
        {
#if LIBHEIF_NUMERIC_VERSION >= BUILD_LIBHEIF_VERSION(1, 2, 0)
            const char *pszContentType =
                heif_image_handle_get_metadata_content_type(m_hImageHandle, id);
            if (pszContentType &&
                EQUAL(pszContentType, "application/rdf+xml") &&
#else
            if (
#endif
                nCount > 0 && nCount < 1024 * 1024)
            {
                std::string osXMP;
                osXMP.resize(nCount);
                heif_image_handle_get_metadata(m_hImageHandle, id, &osXMP[0]);
                if (osXMP.find("<?xpacket") != std::string::npos)
                {
                    char *apszMDList[2] = {&osXMP[0], nullptr};
                    GDALDataset::SetMetadata(apszMDList, "xml:XMP");
                }
            }
        }
        else if (pszType && EQUAL(pszType, "uri ")) 
        {
            // TODO: version check
            const char *pszItemUriType = heif_image_handle_get_metadata_item_uri_type(m_hImageHandle, id);
            if (pszItemUriType)
            {
                if (EQUAL(pszItemUriType, "urn:uuid:aac8ab7d-f519-5437-b7d3-c973d155e253"))
                {
                    std::string osContentID;
                    osContentID.resize(nCount);
                    heif_image_handle_get_metadata(m_hImageHandle, id, &osContentID[0]);
                    char *apszMDList[2] = {&osContentID[0], nullptr};
                    GDALDataset::SetMetadata(apszMDList, "GIMI ContentID");
                }
                else if (EQUAL(pszItemUriType, "urn:nsg:KLV:ul:060E2B34.020B0101.0E010301.01000000"))
                {
                    ST0601 *parser = new ST0601();
                    char **papszMD = nullptr;
                    std::vector<GByte> data(nCount);
                    heif_image_handle_get_metadata(m_hImageHandle, id, &data[0]);
                    size_t dataOffset = 0;
                    double lat1, lon1, lat2, lon2, lat3, lon3, lat4, lon4;
                    bool haveLat1 = false;
                    bool haveLon1 = false;
                    bool haveLat2 = false;
                    bool haveLon2 = false;
                    bool haveLat3 = false;
                    bool haveLon3 = false;
                    bool haveLat4 = false;
                    bool haveLon4 = false;
                    // Note: this assumes that only the body of the set is here.
                    // TODO: We could check for the UL and set length too.
                    while (dataOffset < nCount) {
                        // TODO: get BER OID instead
                        int tag = data[dataOffset];
                        dataOffset += 1;
                        const char *pszTagName = parser->lookupTagName(tag);
                        const char *pszValue = parser->decodeValue(tag, data, &dataOffset);
                        papszMD = CSLAddNameValue(papszMD, pszTagName, pszValue);
                        if (tag == 82)
                        {
                            haveLat1 = true;
                            lat1 = CPLAtof(pszValue);
                        }
                        if (tag == 83)
                        {
                            haveLon1 = true;
                            lon1 = CPLAtof(pszValue);
                        }
                        if (tag == 84)
                        {
                            haveLat2 = true;
                            lat2 = CPLAtof(pszValue);
                        }
                        if (tag == 85)
                        {
                            haveLon2 = true;
                            lon2 = CPLAtof(pszValue);
                        }
                        if (tag == 86)
                        {
                            haveLat3 = true;
                            lat3 = CPLAtof(pszValue);
                        }
                        if (tag == 87)
                        {
                            haveLon3 = true;
                            lon3 = CPLAtof(pszValue);
                        }
                        if (tag == 88)
                        {
                            haveLat4 = true;
                            lat4 = CPLAtof(pszValue);
                        }
                        if (tag == 89)
                        {
                            haveLon4 = true;
                            lon4 = CPLAtof(pszValue);
                        }
                    }
                    if (haveLat1 && haveLon1 && haveLat2 && haveLon2 && haveLat3 && haveLon3 && haveLat4 && haveLon4)
                    {
                        nGCPCount = 4;
                        pasGCPList = (GDAL_GCP *)CPLCalloc(sizeof(GDAL_GCP), nGCPCount);
                        GDALInitGCPs(nGCPCount, pasGCPList);
                        pasGCPList[0].pszId = CPLStrdup("GCP_1");
                        pasGCPList[0].dfGCPX = lon1;
                        pasGCPList[0].dfGCPY = lat1;
                        pasGCPList[0].dfGCPPixel = 0;
                        pasGCPList[0].dfGCPLine = 0;
                        pasGCPList[1].pszId = CPLStrdup("GCP_2");
                        pasGCPList[1].dfGCPX = lon2;
                        pasGCPList[1].dfGCPY = lat2;
                        pasGCPList[1].dfGCPPixel = nRasterXSize;
                        pasGCPList[1].dfGCPLine = 0;
                        pasGCPList[2].pszId = CPLStrdup("GCP_3");
                        pasGCPList[2].dfGCPX = lon3;
                        pasGCPList[2].dfGCPY = lat3;
                        pasGCPList[2].dfGCPPixel = nRasterXSize;
                        pasGCPList[2].dfGCPLine = nRasterYSize;
                        pasGCPList[3].pszId = CPLStrdup("GCP_4");
                        pasGCPList[3].dfGCPX = lon4;
                        pasGCPList[3].dfGCPY = lat4;
                        pasGCPList[3].dfGCPPixel = 0;
                        pasGCPList[3].dfGCPLine = nRasterYSize;
                        if (GDALGCPsToGeoTransform(nGCPCount, pasGCPList, adfGeoTransform, FALSE))
                        {
                            bGeoTransformSet = TRUE;
                        }
                        else
                        {
                            printf("failed to transform\n");
                        }
                    }
                    GDALDataset::SetMetadata(papszMD, "GIMI ST0601");
                    delete parser;
                }
                else if (EQUAL(pszItemUriType, "urn:nsg:KLV:ul:060E2B34.02050101.0E010504.00000000"))
                {
                    printf("TODO: process MIMD metadata\n");
                }
                else
                {
                    printf("TODO: process other URI metadata: %s\n", pszItemUriType);
                }
            }
        }
    }
}


/************************************************************************/
/*                         OpenThumbnails()                             */
/************************************************************************/

void GDALHEIFDataset::OpenThumbnails()
{
    int nThumbnails =
        heif_image_handle_get_number_of_thumbnails(m_hImageHandle);
    if (nThumbnails <= 0)
        return;

    heif_item_id thumbnailId = 0;
    heif_image_handle_get_list_of_thumbnail_IDs(m_hImageHandle, &thumbnailId,
                                                1);
    heif_image_handle *hThumbnailHandle = nullptr;
    heif_image_handle_get_thumbnail(m_hImageHandle, thumbnailId,
                                    &hThumbnailHandle);
    if (hThumbnailHandle == nullptr)
        return;

    const int nThumbnailBands =
        3 + (heif_image_handle_has_alpha_channel(hThumbnailHandle) ? 1 : 0);
    if (nThumbnailBands != nBands)
    {
        heif_image_handle_release(hThumbnailHandle);
        return;
    }
#if LIBHEIF_NUMERIC_VERSION >= BUILD_LIBHEIF_VERSION(1, 4, 0)
    const int nBits =
        heif_image_handle_get_luma_bits_per_pixel(hThumbnailHandle);
    if (nBits != heif_image_handle_get_luma_bits_per_pixel(m_hImageHandle))
    {
        heif_image_handle_release(hThumbnailHandle);
        return;
    }
#endif

    auto poOvrDS = cpl::make_unique<GDALHEIFDataset>();
    poOvrDS->m_hImageHandle = hThumbnailHandle;
    poOvrDS->m_bIsThumbnail = true;
    poOvrDS->nRasterXSize = heif_image_handle_get_width(hThumbnailHandle);
    poOvrDS->nRasterYSize = heif_image_handle_get_height(hThumbnailHandle);
    for (int i = 0; i < nBands; i++)
    {
        poOvrDS->SetBand(i + 1, new GDALHEIFRasterBand(poOvrDS.get(), i + 1));
    }
    m_apoOvrDS.push_back(std::move(poOvrDS));
}

/************************************************************************/
/*                              Open()                                  */
/************************************************************************/

GDALDataset *GDALHEIFDataset::Open(GDALOpenInfo *poOpenInfo)
{
    if (!Identify(poOpenInfo))
        return nullptr;
    if (poOpenInfo->eAccess == GA_Update)
    {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "Update of existing HEIF file not supported");
        return nullptr;
    }

    auto poDS = cpl::make_unique<GDALHEIFDataset>();
    if (!poDS->Init(poOpenInfo))
        return nullptr;

    return poDS.release();
}

/************************************************************************/
/*                          GetGeoTransform()                           */
/************************************************************************/

CPLErr GDALHEIFDataset::GetGeoTransform(double *padfTransform)

{
    memcpy(padfTransform, adfGeoTransform, sizeof(double) * 6);

    if (bGeoTransformSet)
        return CE_None;

    return CE_Failure;
}

/************************************************************************/
/*                          GetSpatialRef()                             */
/************************************************************************/

const OGRSpatialReference *GDALHEIFDataset::GetSpatialRef() const

{
    if (bGeoTransformSet)
        return &m_oGCPSRS;

    return nullptr;
}

/************************************************************************/
/*                            GetGCPCount()                             */
/************************************************************************/

int GDALHEIFDataset::GetGCPCount()

{
    return nGCPCount;
}

/************************************************************************/
/*                          GetGCPSpatialRef()                          */
/************************************************************************/

const OGRSpatialReference *GDALHEIFDataset::GetGCPSpatialRef() const

{
    if (nGCPCount > 0)
        return &m_oGCPSRS;

    return nullptr;
}

/************************************************************************/
/*                               GetGCP()                               */
/************************************************************************/

const GDAL_GCP *GDALHEIFDataset::GetGCPs()

{
    return pasGCPList;
}


/************************************************************************/
/*                          GDALHEIFRasterBand()                        */
/************************************************************************/

GDALHEIFRasterBand::GDALHEIFRasterBand(GDALHEIFDataset *poDSIn, int nBandIn)
{
    poDS = poDSIn;
    nBand = nBandIn;

    eDataType = GDT_Byte;
#if LIBHEIF_NUMERIC_VERSION >= BUILD_LIBHEIF_VERSION(1, 4, 0)
    const int nBits =
        heif_image_handle_get_luma_bits_per_pixel(poDSIn->m_hImageHandle);
    if (nBits > 8)
    {
        eDataType = GDT_UInt16;
    }
    if (nBits != 8 && nBits != 16)
    {
        GDALRasterBand::SetMetadataItem("NBITS", CPLSPrintf("%d", nBits),
                                        "IMAGE_STRUCTURE");
    }
#endif
    nBlockXSize = poDS->GetRasterXSize();
    nBlockYSize = 1;
}

/************************************************************************/
/*                            IReadBlock()                              */
/************************************************************************/

CPLErr GDALHEIFRasterBand::IReadBlock(int, int nBlockYOff, void *pImage)
{
    GDALHEIFDataset *poGDS = static_cast<GDALHEIFDataset *>(poDS);
    if (poGDS->m_bFailureDecoding)
        return CE_Failure;
    const int nBands = poGDS->GetRasterCount();
    if (poGDS->m_hImage == nullptr)
    {
        auto err = heif_decode_image(
            poGDS->m_hImageHandle, &(poGDS->m_hImage), heif_colorspace_RGB,
            nBands == 3
                ? (
#if LIBHEIF_NUMERIC_VERSION >= BUILD_LIBHEIF_VERSION(1, 4, 0)
                      eDataType == GDT_UInt16 ?
#if CPL_IS_LSB
                                              heif_chroma_interleaved_RRGGBB_LE
#else
                                              heif_chroma_interleaved_RRGGBB_BE
#endif
                                              :
#endif
                                              heif_chroma_interleaved_RGB)
                : (
#if LIBHEIF_NUMERIC_VERSION >= BUILD_LIBHEIF_VERSION(1, 4, 0)
                      eDataType == GDT_UInt16
                          ?
#if CPL_IS_LSB
                          heif_chroma_interleaved_RRGGBBAA_LE
#else
                          heif_chroma_interleaved_RRGGBBAA_BE
#endif
                          :
#endif
                          heif_chroma_interleaved_RGBA),
            nullptr);
        if (err.code != heif_error_Ok)
        {
            CPLError(CE_Failure, CPLE_AppDefined, "%s",
                     err.message ? err.message : "Cannot decode image");
            poGDS->m_bFailureDecoding = true;
            return CE_Failure;
        }
        const int nBitsPerPixel = heif_image_get_bits_per_pixel(
            poGDS->m_hImage, heif_channel_interleaved);
        if (nBitsPerPixel != nBands * GDALGetDataTypeSize(eDataType))
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Unexpected bits_per_pixel = %d value", nBitsPerPixel);
            poGDS->m_bFailureDecoding = true;
            return CE_Failure;
        }
    }

    int nStride = 0;
    const uint8_t *pSrcData = heif_image_get_plane_readonly(
        poGDS->m_hImage, heif_channel_interleaved, &nStride);
    pSrcData += nBlockYOff * nStride;
    if (eDataType == GDT_Byte)
    {
        for (int i = 0; i < nBlockXSize; i++)
            (static_cast<GByte *>(pImage))[i] =
                pSrcData[nBand - 1 + i * nBands];
    }
    else
    {
        for (int i = 0; i < nBlockXSize; i++)
            (static_cast<GUInt16 *>(pImage))[i] =
                (reinterpret_cast<const GUInt16 *>(
                    pSrcData))[nBand - 1 + i * nBands];
    }

    return CE_None;
}

/************************************************************************/
/*                       GDALRegister_HEIF()                            */
/************************************************************************/

void GDALRegister_HEIF()

{
    if (!GDAL_CHECK_VERSION("HEIF driver"))
        return;

    if (GDALGetDriverByName("HEIF") != nullptr)
        return;

    GDALDriver *poDriver = new GDALDriver();

    poDriver->SetDescription("HEIF");
    poDriver->SetMetadataItem(GDAL_DCAP_RASTER, "YES");
    poDriver->SetMetadataItem(
        GDAL_DMD_LONGNAME,
        "ISO/IEC 23008-12:2017 High Efficiency Image File Format");
    poDriver->SetMetadataItem(GDAL_DMD_MIMETYPE, "image/heic");
    poDriver->SetMetadataItem(GDAL_DMD_HELPTOPIC, "drivers/raster/heif.html");
    poDriver->SetMetadataItem(GDAL_DMD_EXTENSION, "heic");
#ifdef HAS_CUSTOM_FILE_READER
    poDriver->SetMetadataItem(GDAL_DCAP_VIRTUALIO, "YES");
#endif

    poDriver->pfnOpen = GDALHEIFDataset::Open;
    poDriver->pfnIdentify = GDALHEIFDataset::Identify;

    poDriver->SetMetadataItem("LIBHEIF_VERSION", LIBHEIF_VERSION);

    GetGDALDriverManager()->RegisterDriver(poDriver);
}
