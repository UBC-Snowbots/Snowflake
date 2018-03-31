/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Class Implementation of DBSCAN (Density-based clustering)
 */

#include <DBSCAN.h>

DBSCAN::DBSCAN(int min_neighbours, float radius, unsigned int num_threads) {
    this->_min_neighbors = min_neighbours;
    this->_radius        = radius;
    this->_num_threads = num_threads;
    this->_clusters      = vector<pcl::PointCloud<pcl::PointXYZ>>();
}

void DBSCAN::setMinNeighbours(int new_min_neighour) {
    this->_min_neighbors = new_min_neighour;
}

void DBSCAN::setRadius(float new_radius) {
    this->_radius = new_radius;
}

vector<pcl::PointCloud<pcl::PointXYZ>>
DBSCAN::findClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr) {
    this->_pcl = *pclPtr;

    findNeighbors();

    for (unsigned int i = 0; i < this->_pcl.size(); i++) {
        if (isPointVisited(i)) { continue; }
        if (isCore(i)) {
            // create a new cluster, and assign current point to it
            pcl::PointCloud<pcl::PointXYZ> cluster =
            pcl::PointCloud<pcl::PointXYZ>();
            pcl::PointXYZ current_point = this->_pcl[i];
            cluster.push_back(current_point);
            this->_clustered.insert({i, true});

            // expand the cluster, centering on current point
            expand(i, cluster);

            // add this cluster to the list of clusters
            this->_clusters.push_back(cluster);
        }
    }

    return this->_clusters;
}

bool DBSCAN::isCore(unsigned int center_index) {
    return this->_neighbors[center_index].size() >= this->_min_neighbors;
}

void DBSCAN::expand(unsigned int center_index,
                    pcl::PointCloud<pcl::PointXYZ>& cluster) {
    this->_expanded.insert({center_index, true});

    vector<unsigned int> neighbors = this->_neighbors[center_index];

    // iterate through all the neighbors of the point
    for (unsigned int i = 0; i < neighbors.size(); i++) {
        unsigned int current_index  = neighbors[i];
        pcl::PointXYZ current_point = this->_pcl[current_index];

        // add neighbor point to the cluster if it hasn't already been clustered
        if (!isPointVisited(current_index)) {
            cluster.push_back(current_point);
            this->_clustered.insert({current_index, true});
        }

        // expand on the neighbour point if it is a core and it hasn't been
        // expanded yet
        if (!isPointExpanded(current_index) && isCore(current_index)) {
            expand(current_index, cluster);
            this->_expanded.insert({current_index, true});
        }
    }

    return;
}

void DBSCAN::findNeighbors() {
    this->_neighbors = new vector<unsigned int>[this->_pcl.size()];

    // only perform multi-threading if the point cloud is large enough,
    // otherwise there will be more overhead in creating threads than
    // speed up from parallelism
    if (this->_pcl.size() > SEQUENTIAL_CUT_OFF || this->_num_threads > 1) {
        findNeighborsParallel();
    } else {
        findNeighborsSequential();
    }
}

void DBSCAN::findNeighborsSequential() {
    for (unsigned int i = 0; i < this->_pcl.size(); i++) {
        vector<unsigned int> neighbors;
        pcl::PointXYZ current_point = this->_pcl[i];

        // for current_point, determine all neighbour points that are within
        // predetermined radius
        for (unsigned int j = 0; j < this->_pcl.size(); j++) {
            if (i == j) continue;
            pcl::PointXYZ neighbor_point = this->_pcl[j];
            if (dist(current_point, neighbor_point) <= this->_radius) {
                neighbors.push_back(j);
            }
        }

        // store the list of neighbours
        this->_neighbors[i] = neighbors;
    }
}

void DBSCAN::findNeighborsParallel() {
    // array to store this->num_threads number of threads
    auto* threads = new pthread_t[this->_num_threads];

    // calculate how many points in the point cloud each thread should process
    unsigned int points_per_thread =
            (this->_pcl.size() + this->_num_threads - 1) / this->_num_threads;

    // create threads
    for (unsigned int i = 0; i < this->_num_threads; i++) {
        // create argument to pass in to the thread worker
        findNeighborsThreadArg* arg = buildFindNeighborsThreadArg(i, points_per_thread);

        if (pthread_create(threads + i, NULL, findNeighborsThread, arg)) {
            std::cerr << "failed to start worker" << std::endl;
        }
    }

    // join threads
    for (unsigned int i = 0; i < this->_num_threads; i++) {
        if (pthread_join(threads[i], NULL)) {
            std::cerr << "failed to join worker" << std::endl;
        }
    }
}

DBSCAN::findNeighborsThreadArg *DBSCAN::buildFindNeighborsThreadArg(unsigned int thread_index, unsigned int points_per_thread) {
    findNeighborsThreadArg* arg = new findNeighborsThreadArg;

    arg->pcl_pointer       = &this->_pcl;
    arg->neighbors_pointer = this->_neighbors;

    // determine the subset of points each thread should process
    arg->start_index       = thread_index * points_per_thread;
    arg->stop_index        = arg->start_index + points_per_thread - 1;
    if (arg->stop_index >= this->_pcl.size()) {
        arg->stop_index = this->_pcl.size() - 1;
    }

    arg->radius = this->_radius;

    return arg;
}

void* DBSCAN::findNeighborsThread(void* arg) {
    auto* a = (findNeighborsThreadArg*) arg;

    pcl::PointCloud<pcl::PointXYZ> pcl      = *(a->pcl_pointer);
    unsigned int start                      = a->start_index;
    unsigned int stop                       = a->stop_index;
    float radius                            = a->radius;
    vector<unsigned int>* neighbors_storage = a->neighbors_pointer;

    for (unsigned int i = start; i <= stop; i++) {
        pcl::PointXYZ current_point = pcl[i];

        vector<unsigned int> neighbors;

        // for current_point, determine all neighbour points that are within
        // predetermined radius
        for (unsigned int j = 0; j < pcl.size(); j++) {
            if (i == j) continue;
            pcl::PointXYZ neighbor_point = pcl[j];

            if (dist(current_point, neighbor_point) <= radius) {
                neighbors.push_back(j);
            }
        }

        // store the list of neighbours
        neighbors_storage[i] = neighbors;
    }

    // arg pointer is no longer needed, so free memory
    free(arg);

    return NULL;
};

double DBSCAN::dist(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    double dx = abs(p1.x - p2.x);
    double dy = abs(p1.y - p2.y);
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

bool DBSCAN::isPointVisited(unsigned int p_index) {
    return this->_clustered.find(p_index) != this->_clustered.end();
}

bool DBSCAN::isPointExpanded(unsigned int p_index) {
    return this->_expanded.find(p_index) != this->_expanded.end();
}
