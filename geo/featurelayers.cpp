/**
 * featurelayers.cpp
 */

#include "./featurelayers.hpp"

#include "utility/expect.hpp"

#include <ogrsf_frmts.h>

namespace geo {
    
namespace ut = utility;

/** class FeatureLayers */

void FeatureLayers::load(::GDALDataset &dataset
    , boost::optional<const SrsDefinition &> sourceSrs) {
    
    // initialize
    OGRRegisterAll();
    layers.resize(0);
    
    // cycle through layers
    for (int i = 0; i < dataset.GetLayerCount(); i++) {
        
        using boost::format;
        
        // obtain layer
        OGRLayer * ilayer = dataset.GetLayer(i);
        
        // extract srs
        geo::SrsDefinition srcSrs;

        if (sourceSrs) {
            srcSrs = sourceSrs.get();
        } else {
        
            OGRSpatialReference * ogrsr = ilayer->GetSpatialRef();
            ut::expect(ogrsr, "No srs provided in input dataset, and no "
                "override given.");
        
            char * eogrsr;
            ut::expect(ogrsr->exportToProj4(& eogrsr) == OGRERR_NONE,
                "Failed to convert input layer srs to proj4.");
            
            srcSrs = geo::SrsDefinition(
                eogrsr, geo::SrsDefinition::Type::proj4);
        } 
        
        LOG( info3 ) 
            << "Layer " << i << " srs: \""  << srcSrs.string() << "\".";

        // initialize layer
        layers.push_back({ilayer->GetName(), srcSrs});
           
        auto & layer(layers.back());
        uint unsupported(0);
        
        // cycle through features
        OGRFeature * ifeature;
        ilayer->ResetReading();
        uint id = 0;
        
        OGRFeatureDefn *ilayerDefn = ilayer->GetLayerDefn();
        
        while ((ifeature = ilayer->GetNextFeature()) != 0x0) {
            
            bool zDefined;
            
            // extract properties
            Features::Properties properties;
            
            for (int j = 0; j < ilayerDefn->GetFieldCount(); j++)
                properties[ilayerDefn->GetFieldDefn(j)->GetNameRef()] 
                    = ifeature->GetFieldAsString(j);
        
           // extract geometry
           OGRGeometry *igeometry = ifeature->GetGeometryRef();
           
           if ( igeometry == 0x0 ) {
               LOG(warn2) << "Geometry-less feature encountered, skipping.";
               continue;
           }
           
           if (igeometry->getCoordinateDimension() != 2 
               && igeometry->getCoordinateDimension() != 3) {
               
               LOG(warn2) << "Unknown feature dimension, skipping.";
               continue;
           } 
           
           zDefined = (igeometry->getCoordinateDimension() == 3 );
           
           /* point */
           if (wkbFlatten(igeometry->getGeometryType()) == wkbPoint ) {
               
                OGRPoint * ipoint = (OGRPoint *) igeometry;
                
                math::Point3 point(
                    ipoint->getX(), ipoint->getY(), ipoint->getZ());
                layer.features.addPoint( 
                        {(format("%s-%d") % layer.name % id++ ).str(), 
                        point, properties, zDefined});
                    
                layer.updateBB(layer.features.points.back().point);
                continue;
           }
           
           /* line string */
           if (wkbFlatten(igeometry->getGeometryType()) == wkbLineString) {
               
                OGRLineString * ilinestring = (OGRLineString *) igeometry;
                math::Points3 points;
                
                
                for (int k = 0; k < ilinestring->getNumPoints(); k++) {
                    
                        OGRPoint ipoint;
                        ilinestring->getPoint(k, & ipoint);
                        points.push_back(math::Point3(ipoint.getX()
                            , ipoint.getY(), ipoint.getZ()));
                        layer.updateBB(points.back());
                }
                
                layer.features.addLineString(
                        {(format( "%s-%d" ) % layer.name % id++ ).str(), 
                        points, properties, zDefined});
                
                continue;
           }
                   
           /* polygon */        
           if (wkbFlatten(igeometry->getGeometryType()) == wkbPolygon ) {
               
                OGRPolygon * ipolygon = (OGRPolygon *) igeometry;
                Features::MultiPolygon::Polygon polygon;
                
                ipolygon->closeRings(); // probably unnecessary

                OGRLinearRing * extring = ipolygon->getExteriorRing();
                                                
                for (int k = 0; k < extring->getNumPoints() - 1; k++) {
                    
                    OGRPoint ipoint;
                    extring->getPoint(k, &ipoint);
                    polygon.exterior.push_back(math::Point3(
                        ipoint.getX(), ipoint.getY(), ipoint.getZ()));
                    layer.updateBB(polygon.exterior.back());                    
                }
                
                for (int k = 0; k < ipolygon->getNumInteriorRings(); k++) {
                    
                    OGRLinearRing *intring = ipolygon->getInteriorRing(k);
                    polygon.interiors.push_back({});
                  
                    for ( int l = 0; l < intring->getNumPoints() -1; l++ ) {
                        OGRPoint ipoint;
                        intring->getPoint(l, &ipoint);
                        polygon.interiors.back().push_back(
                            math::Point3(
                                ipoint.getX(), ipoint.getY(), ipoint.getZ()));                                            
                    }
                }
                
                layer.features.addMultiPolygon({
                    (format( "%s-%d" ) % layer.name % id++ ).str(),
                    properties, { polygon }, zDefined });
                continue;
           }
               
           /* multipolygon, multilinestring - META */
                   
           /* unknown */        
           LOG(warn2) << format( "Unsupported feature type 0x%X, skipping." ) 
              % igeometry->getGeometryType();
           unsupported++;
        }
        
        // end layer
        if (!unsupported) {
            LOG( info3 ) << format( 
                "%s: %5d points, %5d linestrings, %5d (multi)polygons." ) 
                % layer.name
                % layer.features.points.size()
                % layer.features.linestrings.size()
                % layer.features.multipolygons.size();
        } else {
            LOG( info3 ) << format( 
                "%s: %5d points, %5d linestrings, %5d (multi)polygons, "
                "%5d unsupported features." ) 
                % layer.name
                % layer.features.points.size()
                % layer.features.linestrings.size()
                % layer.features.multipolygons.size()
                % unsupported;            
        }
    }
        
    // all done
}

/* Class FeatureLayers::Layer */


void FeatureLayers::Layer::updateBB( const math::Point3 & point ) {

    if (!featuresBB)
        featuresBB = math::Extents3(point,point);
    else
        update(featuresBB.get(), point);
}        


/** class FeatureLayers::Features::Surface */

void FeatureLayers::Features::Surface::addPatchesFromPolygon(
    const Features::MultiPolygon::Polygon & ) {
    
    // find the polygon principal component (plane) and establish trafo
    // transform polygon to xy plane
    // triangule boundary points
    // build surface by testing against polygon
    // reverse transform to original plane
    // done
}

} // namespace geo