#ifndef __TOPOMETRIC_LOCALIZER__
#define __TOPOMETRIC_LOCALIZER__

#include "localizer_metric.hpp"

namespace dg
{

class TopometricPose
{
public:
    TopometricPose(int _node_id = -1, int _edge_idx = -1, double _dist = 0) : node_id(_node_id), edge_idx(_edge_idx), dist(_dist) { }

    int node_id;

    int edge_idx;

    double dist;
};

class TopometricLocalizer : public MetricLocalizer
{
public:
    TopometricPose getPose() const { return m_pose_topo; }

protected:
    TopometricPose m_pose_topo;
};

} // End of 'dg'

#endif // End of '__TOPOMETRIC_LOCALIZER__'
