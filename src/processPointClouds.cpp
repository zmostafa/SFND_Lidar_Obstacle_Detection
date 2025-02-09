// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>());
    // Points downsampling
    pcl::VoxelGrid<PointT> filterObject; 
    filterObject.setInputCloud(cloud);
    filterObject.setLeafSize(filterRes, filterRes, filterRes);
    filterObject.filter(*filteredCloud);

    typename pcl::PointCloud<PointT>::Ptr regionCloud (new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true); 
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*regionCloud);

    // Removing roof point from display
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(regionCloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices){
        inliers->indices.push_back(point);
    }
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(regionCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*regionCloud);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return regionCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
    for(auto& index : inliers->indices){
        planeCloud->points.push_back(cloud->points[index]);
    }
    // Create extract Object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    // Creat eSegmentation Object
    pcl::SACSegmentation<PointT> segment;
    // Setting object
    segment.setOptimizeCoefficients(true);
    segment.setModelType(pcl::SACMODEL_PLANE);
    segment.setMethodType(pcl::SAC_RANSAC);
    segment.setDistanceThreshold(distanceThreshold);
    segment.setMaxIterations(maxIterations);

    segment.setInputCloud(cloud);
    segment.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0){
        std::cerr << "Could not estimate a planer model for the givien dataset." << std::endl;
    }
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> euc;
    
    euc.setClusterTolerance(clusterTolerance);
    euc.setMinClusterSize(minSize);
    euc.setMaxClusterSize(maxSize);
    euc.setSearchMethod(tree);
    euc.setInputCloud(cloud);
    euc.extract(cluster_indices);

    for(auto& indice : cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for(auto& index : indice.indices){
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac_segmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol){
    auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Re-implement the function
	while(maxIterations--){
		std::unordered_set<int> inliers;
		while(inliers.size() < 3){
			inliers.insert(rand() % (cloud->points.size()));
		}

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		++itr;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		++itr;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		float a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1); //a=i=(y2−y1)(z3−z1)−(z2−z1)(y3−y1)
		float b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1); //b=j=(z2-z1)(x3-x1)-(x2-x1)(z3-z1)
		float c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1); //c=k=(x2−x1)(y3−y1)−(y2−y1)(x3−x1)
		float d = -(a*x1+b*y1+c*z1);

		for(int index = 0 ; index < cloud->points.size(); ++index){
			if(inliers.count(index) > 0)
				continue;
			
			pcl::PointXYZI point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			float dist = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a + b*b + c*c);

			if(dist <= distanceTol)
				inliers.insert(index);
		}
		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}
	}

    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for(int index = 0 ; index < cloud->points.size(); ++index){
        PointT point = cloud->points[index];
        if(inliersResult.count(index)){
            planeCloud->points.push_back(point);
        }else{
            obstacleCloud->points.push_back(point);
        }
    }
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "New RANSAC Implementation took: " << elapsedTime.count() << " milliseconds" << std::endl;
	return {obstacleCloud, planeCloud};
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* kdTree, float distanceTol, int minSize, int maxSize){
    
    auto startTime = std::chrono::steady_clock::now();
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> processed (cloud->points.size(),false);
	
    int index = 0;
	while(index < cloud->points.size()){
		if(processed[index]){
			++index;
			continue;
		}

        std::vector<int> clusterIdx;
		typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
		_clusterHelper(index, cloud, clusterIdx, processed, kdTree, distanceTol);
		
        if(clusterIdx.size() >= minSize && clusterIdx.size() <= maxSize){
            for(int idx = 0; idx < clusterIdx.size(); ++idx){
                cluster->points.push_back(cloud->points[clusterIdx[idx]]);
                processed[clusterIdx[idx]] = true;
            }

            cluster->width = cluster->points.size();
            cluster->height = 1;

            clusters.push_back(cluster);
        }
		++index;
	}
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "New EuclideanClustring Implementation took: " << elapsedTime.count() << " milliseconds" << std::endl;
	return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::_clusterHelper(int indice, const typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster,std::vector<bool>& processed, KdTree* tree, float distanceTol){
    processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearPoints = tree->search(cloud->points[indice],distanceTol);
	for(int id : nearPoints){
		if(!processed[id]){
			_clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
		}
	}
}