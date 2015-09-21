#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/search/kdtree.h>
#include "boost_graph_extension.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointInTPtr;
typedef flann::L1<float> DistT;

 template<typename Type> void convertToFLANN ( const typename pcl::PointCloud<Type>::ConstPtr & cloud, flann::Matrix<float> &data )
{
    data.rows = cloud->points.size ();
    data.cols = sizeof ( cloud->points[0].histogram ) / sizeof ( float ); // number of histogram bins

    flann::Matrix<float> flann_data ( new float[data.rows * data.cols], data.rows, data.cols );

    for ( size_t i = 0; i < data.rows; ++i )
        for ( size_t j = 0; j < data.cols; ++j )
        {
            flann_data.ptr () [i * data.cols + j] = cloud->points[i].histogram[j];
        }

    data = flann_data;
}


void nearestKSearch ( flann::Index<flann::L1<float> > * index, float * descr, int descr_size, int k, flann::Matrix<int> &indices,
                            flann::Matrix<float> &distances )
{
    flann::Matrix<float> p = flann::Matrix<float> ( new float[descr_size], 1, descr_size );
    memcpy ( &p.ptr () [0], &descr[0], p.cols * p.rows * sizeof ( float ) );

    index->knnSearch ( p, indices, distances, k, flann::SearchParams ( 128 ) );
    delete[] p.ptr ();
}


void estimateViewTransformationBySIFT ( const Vertex &src, const Vertex &trgt, Graph &grph, flann::Index<DistT> *flann_index,
                                        Eigen::Matrix4f &transformation, std::vector<Edge> & edges, bool use_gc )
{
    const int K = 1;
    flann::Matrix<int> indices = flann::Matrix<int> ( new int[K], 1, K );
    flann::Matrix<float> distances = flann::Matrix<float> ( new float[K], 1, K );

    boost::shared_ptr< pcl::PointCloud<PointT> > pSiftKeypointsSrc (new pcl::PointCloud<PointT>);
    boost::shared_ptr< pcl::PointCloud<PointT> > pSiftKeypointsTrgt (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*(grph[ src].pScenePCl_f), grph[ src].siftKeypointIndices_, *(pSiftKeypointsSrc ));
    pcl::copyPointCloud(*(grph[trgt].pScenePCl_f), grph[trgt].siftKeypointIndices_, *(pSiftKeypointsTrgt));

    pcl::CorrespondencesPtr temp_correspondences ( new pcl::Correspondences );
    temp_correspondences->resize(pSiftKeypointsSrc->size ());

    for ( size_t keypointId = 0; keypointId < pSiftKeypointsSrc->size (); keypointId++ )
    {
        FeatureT searchFeature = grph[src].pSiftSignatures_->at ( keypointId );
        int size_feat = sizeof ( searchFeature.histogram ) / sizeof ( float );
        nearestKSearch ( flann_index, searchFeature.histogram, size_feat, K, indices, distances );

        pcl::Correspondence corr;
        corr.distance = distances[0][0];
        corr.index_query = keypointId;
        corr.index_match = indices[0][0];
        temp_correspondences->at(keypointId) = corr;
    }

    if(!use_gc)
    {
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr rej;
        rej.reset (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> ());
        pcl::CorrespondencesPtr after_rej_correspondences (new pcl::Correspondences ());

        rej->setMaximumIterations (50000);
        rej->setInlierThreshold (0.02);
        rej->setInputTarget (pSiftKeypointsTrgt);
        rej->setInputSource (pSiftKeypointsSrc);
        rej->setInputCorrespondences (temp_correspondences);
        rej->getCorrespondences (*after_rej_correspondences);

        transformation = rej->getBestTransformation ();
        pcl::registration::TransformationEstimationSVD<PointT, PointT> t_est;
        t_est.estimateRigidTransformation (*pSiftKeypointsSrc, *pSiftKeypointsTrgt, *after_rej_correspondences, transformation);
    }
    else
    {
        pcl::GeometricConsistencyGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB> gcg_alg;

        gcg_alg.setGCThreshold (15);
        gcg_alg.setGCSize (0.01);
        gcg_alg.setInputCloud(pSiftKeypointsSrc);
        gcg_alg.setSceneCloud(pSiftKeypointsTrgt);
        gcg_alg.setModelSceneCorrespondences(temp_correspondences);

        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
        std::vector<pcl::Correspondences> clustered_corrs;
        gcg_alg.recognize(transformations, clustered_corrs);

        for(size_t i=0; i < transformations.size(); i++)
        {
            PointInTPtr transformed_PCl (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud (*grph[src].pScenePCl, *transformed_PCl, transformations[i]);
        }
    }
}

template void convertToFLANN<pcl::Histogram<128> > (const pcl::PointCloud<pcl::Histogram<128> >::ConstPtr & cloud, flann::Matrix<float> &data ); // explicit instantiation.

int main(int argc, char **argv)
{
    bool use_gc_s2s_ = true;
    Vertex *vertexIt;
    Vertex vrtx;
    Graph grph_;
    Eigen::Matrix4f transformation;
    std::vector<Edge> edge;

    flann::Matrix<float> flann_data;
    flann::Index<DistT> *flann_index;
    convertToFLANN<pcl::Histogram<128> > ( grph_[vrtx].pSiftSignatures_, flann_data );
    flann_index = new flann::Index<DistT> ( flann_data, flann::KDTreeIndexParams ( 4 ) );
    flann_index->buildIndex ();
    estimateViewTransformationBySIFT ( *vertexIt, vrtx, grph_, flann_index, transformation, edge, use_gc_s2s_ );
    delete flann_index;

    return EXIT_SUCCESS;
}

