#ifndef __UTM_CONVERTER__
#define __UTM_CONVERTER__

#include "core/basic_type.hpp"

namespace dg
{

class Point2UTM : public Point2
{
public:
    Point2UTM(double _x = 0, double _y = 0, int _zone = 52, bool _is_south = false) : Point2(_x, _y), zone(_zone), is_south(_is_south) { }

    int zone;

    bool is_south;
};

/**
 * @brief A converter from geodesic notation to UTM notation
 *
 * TODO
 */
class UTMConverter
{
public:
    Point2 toMetric(const LatLon& ll) const;

    LatLon toLatLon(const Point2& metric) const;

    bool setReference(const Point2UTM& utm) { m_refer_utm = utm; return true; }

    bool setReference(const LatLon& ll) { return setReference(cvtLatLon2UTM(ll)); }

    Point2UTM getReference() const { return m_refer_utm; }

    static Point2UTM cvtLatLon2UTM(const LatLon& ll);

    static LatLon cvtUTM2LatLon(const Point2UTM& utm);

protected:
    Point2UTM m_refer_utm;
};

} // End of 'dg'

#endif // End of '__UTM_CONVERTER__'
