#include "StateLattice.h"

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/unordered_set.hpp>
#include <cmath>
#include <cstdio>

//#include "cauldron/cauldron.h"
//#include "cauldron/EigenUtils.h"

StateLattice::StateLattice()
 : m_resolution(0.0)
{

}

void
StateLattice::init(double resolution)
{
    m_resolution = resolution;

    generatePrimitivePathSets();
    m_orientationLUT.build(m_primitivePathSets);
}

double
StateLattice::resolution(void) const
{
    return m_resolution;
}

const std::vector<std::vector<PrimitivePathPtr> >&
StateLattice::primitivePathSets(void) const
{
    return m_primitivePathSets;
}

const OrientationLUT&
StateLattice::orientationLUT(void) const
{
    return m_orientationLUT;
}

void
StateLattice::generatePrimitivePathSets(void)
{
    // state lattice is symmetrical about x=0, y=0, and y=x lines
    // hence, we only need to generate paths for 1st quadrant (x >= 0, y >= 0)

    m_primitivePathSets.clear();

    int pathParams[4][3][6] =
    {
        {
            {1,0,1,0,1,0},
//            {1,0,1,0,2,-1},
//            {1,0,1,0,1,0},
//            {1,0,1,0,2,1},
            {1,0,0,0,0,1},
            {1,0,0,0,0,-1} // reverse
        },
        {
            {0,1,0,1,0,1},
//            {1,0,1,0,2,-1},
//            {1,0,1,0,1,0},
//            {1,0,1,0,2,1},
            {0,1,0,0,1,0},
            {0,1,0,0,-1,0} // reverse
        },
        {
            {-1,0,-1,0,-1,0},
//            {1,0,1,0,2,-1},
//            {1,0,1,0,1,0},
//            {1,0,1,0,2,1},
            {-1,0,0,0,0,1},
            {-1,0,0,0,0,-1} // reverse
        },
        {
            {0,-1,0,-1,0,-1},
//            {1,0,1,0,2,-1},
//            {1,0,1,0,1,0},
//            {1,0,1,0,2,1},
            {0,-1,0,0,1,0},
            {0,-1,0,0,-1,0} // reverse
        }
    };

    for (size_t i = 0; i < sizeof(pathParams) / sizeof(pathParams[0]); ++i)
    {
        std::vector<PrimitivePathPtr> set;
        for (size_t j = 0; j < sizeof(pathParams[0]) / sizeof(pathParams[0][0]); ++j)
        {
            PrimitivePathPtr path = boost::make_shared<PrimitivePath>(m_resolution);

            const int* pathParam = pathParams[i][j];

            path->addWaypoint(LatticePose(Eigen::Vector3i::Zero(),
                                          Eigen::Vector2i(pathParam[0], pathParam[1])));
            path->addWaypoint(LatticePose(Eigen::Vector3i(pathParam[2], pathParam[3], 0),
                                          Eigen::Vector2i(pathParam[4], pathParam[5])));

            set.push_back(path);
        }
        m_primitivePathSets.push_back(set);
    }
/*
    // reflect over y=x
    int setCount = m_primitivePathSets.size();
    for (int i = setCount - 1; i >= 0; i--)
    {
        const std::vector<PrimitivePathPtr>& set = m_primitivePathSets.at(i);
        std::vector<PrimitivePathPtr> setReflect;

        for (size_t j = 0; j < set.size(); ++j)
        {
            const PrimitivePathPtr& pathRef = set.at(j);
            if (pathRef->waypoints().front().xYaw() != pathRef->waypoints().front().yYaw())
            {
                PrimitivePathPtr path = boost::make_shared<PrimitivePath>(m_resolution);

                for (size_t k = 0; k < pathRef->waypoints().size(); ++k)
                {
                    const LatticePose& p = pathRef->waypoints().at(k);

                    path->addWaypoint(LatticePose(Eigen::Vector3i(p.y(), p.x(), 0),
                                                  Eigen::Vector2i(p.yYaw(), p.xYaw())));
                }

                setReflect.push_back(path);
            }
        }

        if (!setReflect.empty())
        {
            m_primitivePathSets.push_back(setReflect);
        }
    }

    // reflect over y=0
    setCount = m_primitivePathSets.size();
    for (int i = setCount - 1; i >= 0; i--)
    {
        const std::vector<PrimitivePathPtr>& set = m_primitivePathSets.at(i);
        std::vector<PrimitivePathPtr> setReflect;

        for (size_t j = 0; j < set.size(); ++j)
        {
            const PrimitivePathPtr& pathRef = set.at(j);
            if (pathRef->waypoints().front().xYaw() != 0)
            {
                PrimitivePathPtr path = boost::make_shared<PrimitivePath>(m_resolution);

                for (size_t k = 0; k < pathRef->waypoints().size(); ++k)
                {
                    const LatticePose& p = pathRef->waypoints().at(k);

                    path->addWaypoint(LatticePose(Eigen::Vector3i(-p.x(), p.y(), 0),
                                                  Eigen::Vector2i(-p.xYaw(), p.yYaw())));
                }

                setReflect.push_back(path);
            }
        }

        if (!setReflect.empty())
        {
            m_primitivePathSets.push_back(setReflect);
        }
    }

    // reflect over x=0
    setCount = m_primitivePathSets.size();
    for (int i = setCount - 1; i >= 0; i--)
    {
        const std::vector<PrimitivePathPtr>& set = m_primitivePathSets.at(i);
        std::vector<PrimitivePathPtr> setReflect;

        for (size_t j = 0; j < set.size(); ++j)
        {
            const PrimitivePathPtr& pathRef = set.at(j);
            if (pathRef->waypoints().front().yYaw() != 0)
            {
                PrimitivePathPtr path = boost::make_shared<PrimitivePath>(m_resolution);

                for (size_t k = 0; k < pathRef->waypoints().size(); ++k)
                {
                    const LatticePose& p = pathRef->waypoints().at(k);

                    path->addWaypoint(LatticePose(Eigen::Vector3i(p.x(), -p.y(), 0),
                                                  Eigen::Vector2i(p.xYaw(), -p.yYaw())));
                }

                setReflect.push_back(path);
            }
        }

        if (!setReflect.empty())
        {
            m_primitivePathSets.push_back(setReflect);
        }
    }
*/
#ifdef blah
    fprintf(stderr, "# primitive paths: %zu\n", m_primitivePathSets.size() * m_primitivePathSets.at(0).size());
    for (size_t i = 0; i < m_primitivePathSets.size(); ++i)
    {
        const LatticePose& p0 = m_primitivePathSets.at(i).at(0)->waypoints().front();

        fprintf(stderr, "%zu: %d %d %d %d\n",
                        i, p0.x(), p0.y(), p0.xYaw(), p0.yYaw());

        for (size_t j = 0; j < m_primitivePathSets.at(i).size(); ++j)
        {
            const LatticePose& p1 = m_primitivePathSets.at(i).at(j)->waypoints().back();

            fprintf(stderr, "-> %d %d %d %d\n",
                    p1.x(), p1.y(), p1.xYaw(), p1.yYaw());
        }
    }
#endif
}


