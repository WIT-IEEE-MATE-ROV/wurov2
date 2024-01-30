#!/usr/bin/env python

import Metashape

doc = Metashape.Application.document
doc.open("project.psz")
chunk = doc.chunk
chunk.matchPhotos(downscale=1, generic_preselection=True, reference_preselection=False)
chunk.alignCameras()
chunk.buildDepthMaps(downscale=4, filter_mode=Metashape.AggressiveFiltering)
chunk.buildModel(source_data=Metashape.DepthMapsData, surface_type=Metashape.Arbitrary, interpolation=Metashape.EnabledInterpolation)
chunk.buildUV(mapping_mode=Metashape.GenericMapping)
chunk.buildTexture(blending_mode=Metashape.MosaicBlending, texture_size=4096)
doc.save()