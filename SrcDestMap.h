#ifndef SRC_DEST_MAP_H_
#define SRC_DEST_MAP_H_

#include <climits>

class SrcDestMap
{

private:
    int **map;
    int n;

public:
    SrcDestMap(int num)
    {
        n = num;
        map = new int *[num];
        for (int i = 0; i < n; ++i)
        {
            map[i] = new int[num];
        }
    }

    ~SrcDestMap()
    {
        for (int i = 0; i < n; ++i)
        {
            delete[] map[i];
        }
        delete[] map;
    }

    int *operator [](int i) const
    {
        return map[i];
    }

    int *&operator [](int i)
    {
        return map[i];
    }

    int dimension() {
        return n;
    }

};

#endif