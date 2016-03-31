#!/usr/bin/env python
# -*- coding: utf-8 -*-
###############################################################################
# $Id$
#
# Project:  GDAL/OGR Test Suite
# Purpose:  Test read/write functionality for MBTiles driver.
# Author:   Even Rouault, <even dot rouault at mines dash paris dot org>
#
###############################################################################
# Copyright (c) 2012-2016, Even Rouault <even dot rouault at mines-paris dot org>
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
###############################################################################

import sys
from osgeo import gdal
from osgeo import ogr

sys.path.append( '../pymod' )

import gdaltest

###############################################################################
# Get the mbtiles driver

def mbtiles_1():

    try:
        gdaltest.mbtiles_drv = gdal.GetDriverByName( 'MBTiles' )
    except:
        gdaltest.mbtiles_drv = None

    return 'success'

###############################################################################
# Basic test

def mbtiles_2():

    if gdaltest.mbtiles_drv is None:
        return 'skip'

    if gdal.GetDriverByName( 'JPEG' ) is None:
        return 'skip'

    ds = gdal.OpenEx('data/world_l1.mbtiles', open_options = ['USE_BOUNDS=NO'])
    if ds is None:
        return 'fail'

    if ds.RasterCount != 3:
        gdaltest.post_reason('expected 3 bands')
        return 'fail'

    if ds.GetRasterBand(1).GetOverviewCount() != 1:
        gdaltest.post_reason('did not get expected overview count')
        return 'fail'

    expected_cs_tab = [6324, 19386, 45258]
    expected_cs_tab_jpeg8 = [6016, 13996, 45168]
    for i in range(3):
        cs = ds.GetRasterBand(i + 1).Checksum()
        if ds.GetRasterBand(i + 1).GetColorInterpretation() != gdal.GCI_RedBand + i:
            gdaltest.post_reason('bad color interpretation')
            return 'fail'
        expected_cs = expected_cs_tab[i]
        if cs != expected_cs and cs != expected_cs_tab_jpeg8[i]:
            gdaltest.post_reason('for band %d, cs = %d, different from expected_cs = %d' % (i + 1, cs, expected_cs))
            return 'fail'

    expected_cs_tab = [16642, 15772, 10029]
    expected_cs_tab_jpeg8 = [16621, 14725, 8988]
    for i in range(3):
        cs = ds.GetRasterBand(i + 1).GetOverview(0).Checksum()
        expected_cs = expected_cs_tab[i]
        if cs != expected_cs and cs != expected_cs_tab_jpeg8[i]:
            gdaltest.post_reason('for overview of band %d, cs = %d, different from expected_cs = %d' % (i + 1, cs, expected_cs))
            return 'fail'

    if ds.GetProjectionRef().find('3857') == -1:
        gdaltest.post_reason('projection_ref = %s' % ds.GetProjectionRef())
        return 'fail'

    gt = ds.GetGeoTransform()
    expected_gt = ( -20037508.342789244, 78271.516964020484, 0.0, 20037508.342789244, 0.0, -78271.516964020484 )
    for i in range(6):
        if abs(gt[i] - expected_gt[i]) > 1e-15:
            gdaltest.post_reason('bad gt')
            print(gt)
            print(expected_gt)
            return 'fail'

    md = ds.GetMetadata()
    if md['bounds'] != '-180.0,-85,180,85':
        gdaltest.post_reason('bad metadata')
        return 'fail'

    ds = None

    return 'success'

###############################################################################
# Open a /vsicurl/ DB

def mbtiles_3():

    if gdaltest.mbtiles_drv is None:
        return 'skip'

    try:
        drv = gdal.GetDriverByName( 'HTTP' )
    except:
        drv = None

    if drv is None:
        return 'skip'

    if sys.platform == 'darwin' and gdal.GetConfigOption('TRAVIS', None) is not None:
        print("Hangs on MacOSX Travis sometimes. Not sure why.")
        return 'skip'

    # Check that we have SQLite VFS support
    gdal.PushErrorHandler('CPLQuietErrorHandler')
    ds = ogr.GetDriverByName('SQLite').CreateDataSource('/vsimem/mbtiles_3.db')
    gdal.PopErrorHandler()
    if ds is None:
        return 'skip'
    ds = None
    gdal.Unlink('/vsimem/mbtiles_3.db')

    ds = gdal.Open('/vsicurl/http://a.tiles.mapbox.com/v3/mapbox.geography-class.mbtiles')
    if ds is None:
        # Just skip. The service isn't perfectly reliable sometimes
        return 'skip'

    # long=2,lat=49 in WGS 84 --> x=222638,y=6274861 in Google Mercator
    locationInfo = ds.GetRasterBand(1).GetMetadataItem('GeoPixel_222638_6274861', 'LocationInfo')
    if locationInfo is None or locationInfo.find("France") == -1:
        gdaltest.post_reason('did not get expected LocationInfo')
        print(locationInfo)
        if gdaltest.skip_on_travis():
            return 'skip'
        return 'fail'

    locationInfo2 = ds.GetRasterBand(1).GetOverview(5).GetMetadataItem('GeoPixel_222638_6274861', 'LocationInfo')
    if locationInfo2 != locationInfo:
        gdaltest.post_reason('did not get expected LocationInfo on overview')
        print(locationInfo2)
        if gdaltest.skip_on_travis():
            return 'skip'
        return 'fail'


    return 'success'

###############################################################################
# Basic test without any option

def mbtiles_4():

    if gdaltest.mbtiles_drv is None:
        return 'skip'

    if gdal.GetDriverByName( 'JPEG' ) is None:
        return 'skip'

    ds = gdal.Open('data/world_l1.mbtiles')
    if ds is None:
        return 'fail'

    if ds.RasterCount != 3:
        gdaltest.post_reason('expected 3 bands')
        return 'fail'

    if ds.GetRasterBand(1).GetOverviewCount() != 1:
        gdaltest.post_reason('did not get expected overview count')
        return 'fail'

    if ds.RasterXSize != 512 or ds.RasterYSize != 510:
        gdaltest.post_reason('bad dimensions')
        print(ds.RasterXSize)
        print(ds.RasterYSize)
        return 'fail'

    gt = ds.GetGeoTransform()
    expected_gt = ( -20037508.342789244, 78271.516964020484, 0.0, 19971868.880408563, 0.0, -78271.516964020484 )
    for i in range(6):
        if abs(gt[i] - expected_gt[i]) > 1e-15:
            gdaltest.post_reason('bad gt')
            print(gt)
            print(expected_gt)
            return 'fail'

    ds = None

    return 'success'

###############################################################################
# Test write support of a single band dataset

def mbtiles_5():

    if gdaltest.mbtiles_drv is None:
        return 'skip'

    if gdal.GetDriverByName( 'PNG' ) is None:
        return 'skip'

    src_ds = gdal.Open('data/byte.tif')
    gdaltest.mbtiles_drv.CreateCopy('/vsimem/mbtiles_5.mbtiles', src_ds )
    src_ds = None

    ds = gdal.Open('/vsimem/mbtiles_5.mbtiles')
    if ds.RasterXSize != 19 or ds.RasterYSize != 19:
        gdaltest.post_reason('fail')
        print(ds.RasterXSize)
        print(ds.RasterYSize)
        return 'fail'
    if ds.RasterCount != 2:
        gdaltest.post_reason('fail')
        print(ds.RasterCount)
        return 'fail'
    got_gt = ds.GetGeoTransform()
    expected_gt = (-13095853.550435878, 76.437028285176254, 0.0, 4015708.8887064462, 0.0, -76.437028285176254)
    for i in range(6):
        if abs(expected_gt[i]-got_gt[i])>1e-8:
            gdaltest.post_reason('fail')
            print(got_gt)
            print(expected_gt)
            return 'fail'
    got_cs = ds.GetRasterBand(1).Checksum()
    if got_cs != 4118:
        gdaltest.post_reason('fail')
        print(got_cs)
        return 'fail'
    got_cs = ds.GetRasterBand(2).Checksum()
    if got_cs != 4406:
        gdaltest.post_reason('fail')
        print(got_cs)
        return 'fail'
    got_md = ds.GetMetadata()
    expected_md = {'name': 'mbtiles_5', 'format': 'png', 'bounds': '-117.6420540294745,33.89160566594387,-117.6290077648261,33.90243460427036', 'version': '1.1', 'type': 'overlay', 'description': 'mbtiles_5'}
    if set(got_md.keys()) != set(expected_md.keys()):
        gdaltest.post_reason('fail')
        print(got_md)
        return 'fail'
    for key in got_md:
        if key != 'bounds' and got_md[key] != expected_md[key]:
            gdaltest.post_reason('fail')
            print(got_md)
            return 'fail'
    ds = None

    gdal.Unlink('/vsimem/mbtiles_5.mbtiles')

    return 'success'

###############################################################################
# Test write support with options

def mbtiles_6():

    if gdaltest.mbtiles_drv is None:
        return 'skip'

    if gdal.GetDriverByName( 'JPEG' ) is None:
        return 'skip'

    # Test options
    src_ds = gdal.Open('data/byte.tif')
    options = []
    options += ['TILE_FORMAT=JPEG']
    options += ['QUALITY=50']
    options += ['NAME=name']
    options += ['DESCRIPTION=description']
    options += ['TYPE=baselayer']
    options += ['VERSION=version']
    options += ['WRITE_BOUNDS=no']
    gdaltest.mbtiles_drv.CreateCopy('tmp/mbtiles_6.mbtiles', src_ds, options = options )
    src_ds = None

    ds = gdal.Open('tmp/mbtiles_6.mbtiles')
    got_cs = ds.GetRasterBand(1).Checksum()
    if got_cs == 0:
        gdaltest.post_reason('fail')
        print(got_cs)
        return 'fail'
    got_md = ds.GetMetadata()
    expected_md = {'format': 'jpg', 'version': 'version', 'type': 'baselayer', 'name': 'name', 'description': 'description'}
    if got_md != expected_md:
        gdaltest.post_reason('fail')
        print(got_md)
        return 'fail'
    ds = None

    gdal.Unlink('tmp/mbtiles_6.mbtiles')

    return 'success'

###############################################################################
# Test building overview

def mbtiles_7():

    if gdaltest.mbtiles_drv is None:
        return 'skip'

    if gdal.GetDriverByName( 'PNG' ) is None:
        return 'skip'

    src_ds = gdal.Open('data/small_world.tif')
    data = src_ds.ReadRaster()
    mem_ds = gdal.GetDriverByName('MEM').Create('',
                                                src_ds.RasterXSize * 2,
                                                src_ds.RasterYSize * 2,
                                                src_ds.RasterCount)
    mem_ds.SetProjection(src_ds.GetProjectionRef())
    gt = src_ds.GetGeoTransform()
    gt = [ gt[i] for i in range(6) ]
    gt[1] /= 2
    gt[5] /= 2
    mem_ds.SetGeoTransform(gt)
    mem_ds.WriteRaster(0,0,mem_ds.RasterXSize,mem_ds.RasterYSize,
                       data,src_ds.RasterXSize,src_ds.RasterYSize)
    src_ds = None

    gdaltest.mbtiles_drv.CreateCopy('/vsimem/mbtiles_7.mbtiles', mem_ds, options = ['TILE_FORMAT=PNG8', 'DITHER=YES', 'RESAMPLING=NEAREST'])
    mem_ds = None

    ds = gdal.Open('/vsimem/mbtiles_7.mbtiles', gdal.GA_Update)
    ds.BuildOverviews('NEAR', [2,4])
    ds = None

    ds = gdal.Open('/vsimem/mbtiles_7.mbtiles')
    if ds.GetRasterBand(1).GetOverviewCount() != 1:
        gdaltest.post_reason('fail')
        print(ds.GetRasterBand(1).GetOverviewCount())
        return 'fail'
    expected_ovr_cs = [ 22294, 25695, 6779, 63629 ]
    got_ovr_cs = [ ds.GetRasterBand(i+1).GetOverview(0).Checksum() for i in range(ds.RasterCount) ]
    if expected_ovr_cs != got_ovr_cs:
        gdaltest.post_reason('fail')
        print(got_ovr_cs)
        return 'fail'

    return 'success'

    gdal.Unlink('/vsimem/mbtiles_7.mbtiles')

    return 'success'

###############################################################################
# Cleanup

def mbtiles_cleanup():

    if gdaltest.mbtiles_drv is None:
        return 'skip'

    return 'success'

gdaltest_list = [
    mbtiles_1,
    mbtiles_2,
    mbtiles_3,
    mbtiles_4,
    mbtiles_5,
    mbtiles_6,
    mbtiles_7,
    mbtiles_cleanup ]

#gdaltest_list = [ mbtiles_1, mbtiles_7 ]

if __name__ == '__main__':

    gdaltest.setup_run( 'mbtiles' )

    gdaltest.run_tests( gdaltest_list )

    gdaltest.summarize()
