#ifndef __PATH__
#define __PATH__

namespace dg
{

class Path
{
public:
	/**
	 * The default constructor
	 */
	Path() {}

	/**
	 * Count the number of all points in the path (time complexity: O(1))
	 * @return The number of points
	 */
	int countPoints() const { return (int)m_points.size(); }

	std::list<ID> m_points;
};

} // End of 'dg'

#endif // End of '__PATH__'
