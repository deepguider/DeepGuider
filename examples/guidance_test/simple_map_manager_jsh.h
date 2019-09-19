#pragma once
#ifndef __SIMPLE_MAP_MANAGER__
#define __SIMPLE_MAP_MANAGER__
//This is made by Seohyun. Only temporary use.

#include <vector>
#include "opencx.hpp"
#include "dg_core.hpp"

using namespace std;
using namespace cv;

namespace dg
{

	class NodeInfo
	{
	public:
		NodeInfo(uint64_t _id = 0, double _lon = -1, double _lat = -1, int _type = 0, int _floor = 0) : id(_id), lon(_lon), lat(_lat), type(_type), floor(_floor) { }

		bool operator==(const NodeInfo& rhs) const { return (id == rhs.id); }

		bool operator!=(const NodeInfo& rhs) const { return (id != rhs.id); }

		uint64_t id;
		double lon;
		double lat;
		int type; //기본노드(0), 교차점(1), 출입문/출입구(2), 엘리베이터(3)
		int floor;
		std::vector<uint64_t> sv_ids;
		//std::vector<std::string> pois;
	};

	class EdgeInfo
	{
	public:
		EdgeInfo(uint64_t _id = 0, double _width = 3, double _length = 10, int _type = 0) : id(_id), width(_width), length(_length), type(_type) { }

		uint64_t id; //00010002 => Node 1과 2를 연결
		double width;
		double length;
		int type;
	};

	class Map : public DirectedGraph<NodeInfo, EdgeInfo>
	{
	public:
		/**
		 * The default constructor
		 */
		Map();

		int long2tilex(double lon, int z);

		int lat2tiley(double lat, int z);

		double tilex2long(int x, int z);

		double tiley2lat(int y, int z);

		void downloadMap(cv::Point2i tile);

		cv::Point2i lonlat2xy(double lon, double lat, int z);

		/**
		 * Read a map from the given file
		 * @param lon longitude
		 * @param lat latitude
		 * @param z zoom
		 * @return Result of success (true) or failure (false)
		 */
		bool load(double lon = 128, double lat = 38, int z = 19);

		/**
		 * Check whether this map is empty or not
		 * @return True if empty (true) or not (false)
		 */
		bool isEmpty() const;
	};

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
		//int countPoints() const { return m_points.size(); }
		int countPoints() const { return m_nodes.size(); }

		//std::list<cv::Point2d> m_points;
		std::vector<NodeInfo> m_nodes;
	};

	class MapManager : public DirectedGraph<NodeInfo, EdgeInfo>
	{
	public:
		/**
		 * The default constructor
		 */
		MapManager() {}

		bool generatePath();
		Path getPath(const char* filename = "test_simple_Path.json");

		Map getMap(Path path);
		Map getMap(double lon = 128, double lat = 38, int z = 19);

		std::vector<cv::Point2d> getPOIloc(const char* poiname = "UST");

	protected:
		Map m_map;
		Path m_path;

		double m_lon = 0;
		double m_lat = 0;
		int m_z = 0;
	};

} // End of 'dg'

#endif // End of '__SIMPLE_MAP_MANAGER__'