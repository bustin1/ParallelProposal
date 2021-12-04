
#ifdef _KMEANS_H
#define _KMEANS_H

class Kmeans {
private:

    int* centerX;
    int* centerY;

public:
    
    Kmeans(int k);

    void rand_centers(int* particleLocations);

    void find_clusters();
}

#endif
