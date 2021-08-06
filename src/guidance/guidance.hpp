#ifndef __GUIDANCE__
#define __GUIDANCE__

#include "dg_core.hpp"
#include "core/shared_date.hpp"

namespace dg
{
    class GuidanceManager
    {
    public:
        /** Status of guidance system */
        enum class GuideStatus
        {
            //from localizer info
            GUIDE_INITIAL = 0,
            GUIDE_NORMAL,
            GUIDE_NEAR_ARRIVAL,	//almost arrived 
            GUIDE_ARRIVED,		//finally arrived 
            GUIDE_OOP_DETECT,	//may be out of path. let's see
            GUIDE_OOP,			//out of path, localization confidence is high
            GUIDE_LOST,			//localization confidence is low

            //from exploration control module
            GUIDE_RECOVERY,		//return to previous known POI
            GUIDE_EXPLORATION, 	//find near POI
            GUIDE_OPTIMAL_VIEW,	//find optimal viewpoint to look POI

            //from robot
            GUIDE_LOW_BATTERY,	//Low battery
            GUIDE_PATH_BLOCKED, //Path exist, but temporary blocked
            GUIDE_NO_PATH,		//Path not exist as map
            GUIDE_BAD_GROUND,	//Uneven ground
            GUIDE_ROBOT_FAIL, 	//Robot failure
            GUIDE_FAIL_UNKNOWN,	//Unknown failure

            GUIDE_NOPATH,
            GUIDE_UNKNOWN,
            //	GUIDE_TURNBACK,

            TYPE_NUM			//The number of GuideStatus
        };

        /** Moving status based on localization info*/
        enum class MovingStatus
        {
            ON_NODE = 0,		//The robot has arrived at the node
            ON_EDGE,			//The robot is 0~90% of the edge distance
            APPROACHING_NODE,	//The robot is 90~99% of the edge distance
            STOP_WAIT,			//not moving

            TYPE_NUM			//The number of MovingStatus 
        };

        /** A motion of robot for guidance*/
        enum class Motion
        {
            //on sidewalk
            GO_FORWARD = 0,
            TURN_LEFT,
            TURN_RIGHT,
            TURN_BACK,	    //in case of lost
            STOP,	        //in front of crosswalk

            //on crosswalk
            CROSS_FORWARD,
            CROSS_LEFT,
            CROSS_RIGHT,

            //building
            ENTER_FORWARD,
            ENTER_LEFT,
            ENTER_RIGHT,
            EXIT_FORWARD,
            EXIT_LEFT,
            EXIT_RIGHT,

            TYPE_NUM        //The number of Motion types */
        };

        // Motion mode	
        enum class MotionMode
        {
            MOVE_NORMAL,
            MOVE_CAUTION,
            MOVE_CAUTION_CHILDREN,
            MOVE_CAUTION_CAR,

            TYPE_NUM        //The number of MotionMode types
        };

        /**
        * A realtime action of robot including motion and direction
        * Example:
        * - Go: [GO_**] on [edge_type] with [MotionMode]
        * - TURN: [TURN_**] for [degree] degree on [node_type]
        */
        struct Action
        {
            Action() {}
            Action(Motion _cmd, int _ntype, int _etype, int _degree, MotionMode _mode, double _distance = -1)
                : cmd(_cmd), node_type(_ntype), edge_type(_etype), degree(_degree), mode(_mode), distance(_distance) {}
            Motion cmd = Motion::GO_FORWARD;	// action command
            double distance = -1; //distance for active exploration (it is only used for optimal viewpoint estimation)
            int node_type;
            int edge_type;	//
            int degree = 0;	// additional action direction of turn cmd -180~180
            MotionMode mode = MotionMode::MOVE_NORMAL;	// addition motion info
        };

        /**
        * @brief Detailed guide for each status while robot is moving
        * Guide component: "until where" "in which direction" "with mode"
        * Each of guide component is related to "NodeType" "Direction" "EdgeType"
        */
        struct Guidance
        {
            GuideStatus guide_status;	// current guidance status
            MovingStatus moving_status;	// current moving status
            std::vector<Action> actions;// action command
            ID heading_node_id;	        // heading node
            double relative_angle;
            double distance_to_remain;  // distance to heading node (unit: meter)
            std::string msg;		    // string guidance message
            bool announce = 1;
        };

    protected:
        /**
        * @brief A segment of guided path
        */
        struct ExtendedPathElement : public Point2
        {
            ExtendedPathElement() {};

            /**
            * A constructor with path's segment
            * @param _Current NODE
            * @param _Current EDGE
            * @param _Next NODE
            * @param _Next EDGE
            * @param _Degree of rotation angle to next node. [deg] -180~180
            */
            ExtendedPathElement(ID _fromnode, ID _curedge, ID _tonode,
                ID _nextedge, int _degree = 0, double _x = 0, double _y = 0)
                : cur_node_id(_fromnode), cur_edge_id(_curedge), next_node_id(_tonode),
                next_edge_id(_nextedge), cur_degree(_degree), Point2(_x, _y) {}

            /** Current NODE*/
            ID cur_node_id;

            /** Current EDGE*/
            ID cur_edge_id;

            /** Next NODE*/
            ID next_node_id;

            /** Next EDGE*/
            ID next_edge_id;

            /** Rotation angle to next node. [deg] -180~180 */
            int cur_degree;

            /** junction of guidance */
            bool is_junction = false;
            double remain_distance_to_next_junction = 0;
            double past_distance_from_prev_junction = 0;
            ID next_guide_node_id = 0;
            ID next_guide_edge_id = 0;
        };

    public:
        GuidanceManager() { }

        bool initialize(dg::SharedInterface* shared);

        bool initiateNewGuidance();

        bool update(TopometricPose pose, double confidence);
        bool applyPoseGPS(LatLon gps);

        GuideStatus getGuidanceStatus() const { return m_gstatus; };
        Guidance getGuidance() const { return m_curguidance; };

    protected:
        SharedInterface* m_shared = nullptr;
        Map* getMap() { if (m_shared) return m_shared->getMap(); return nullptr; }
        Path* getPath() { if (m_shared) return m_shared->getPath(); return nullptr; }

        bool validatePath(const Path& path, const Map& map);
        int getDegree(Point2 p1, Point2 p2, Point2 p3);

        std::vector <ExtendedPathElement> m_extendedPath;
        int m_guide_idx = -1;	//starts with -1 because its pointing current guide.

        bool buildGuides();
        ExtendedPathElement getCurExtendedPath(int idx);
        Action setActionTurn(ID nid_cur, ID eid_cur, int degree);
        Action setActionGo(ID nid_next, ID eid_cur, int degree = 0);

    private:
        LatLon m_latlon;
        double m_edge_progress = 0.0;
        double m_rmdistance = 0.0;
        int m_cur_head_degree = 0;
        TopometricPose  m_curpose;
        double m_confidence;
        MovingStatus  m_mvstatus = MovingStatus::ON_EDGE;
        GuideStatus  m_gstatus = GuideStatus::GUIDE_NORMAL;
        Guidance m_curguidance;
        std::vector<Guidance> m_past_guides;
        time_t oop_start = 0, oop_end = 0;
        int m_finalTurn = 0;
        int m_finalEdgeId = 0;
        double m_approachingThreshold = 10.0;
        bool m_arrival = false;
        bool m_juctionguide = true;

        std::string m_movestates[4] = { "ON_NODE","ON_EDGE", "APPROACHING_NODE", "STOP_WAIT" };
        std::string m_nodes[6] = { "POI", "JUNCTION", "DOOR", "ELEVATOR"
            "ESCALATOR", "UNKNOWN" };
        std::string m_edges[7] = { "SIDEWALK", "ROAD", "CROSSWALK", "ELEVATOR",
            "ESCALATOR", "STAIR", "UNKNOWN" };
        std::string m_motions[15] = { "GO_FORWARD", "TURN_LEFT", "TURN_RIGHT",
            "TURN_BACK", "STOP", "CROSS_FORWARD", "CROSS_LEFT",
            "CROSS_RIGHT", "ENTER_FORWARD", "ENTER_LEFT", "ENTER_RIGHT",
            "EXIT_FORWARD", "EXIT_LEFT", "EXIT_RIGHT", "UNKNOWN" };
        std::string m_modes[4] = { "MOVE_NORMAL", "MOVE_CAUTION", "MOVE_CAUTION_CHILDREN",
            "MOVE_CAUTION_CAR" };

        MotionMode getMode(int etype)
        {
            MotionMode mode;
            switch (etype)
            {
            case Edge::EDGE_ROAD:
                mode = MotionMode::MOVE_CAUTION_CAR;
                break;
            case Edge::EDGE_CROSSWALK:
                mode = MotionMode::MOVE_CAUTION;
                break;
            default:
                mode = MotionMode::MOVE_NORMAL;
                break;
            }
            return mode;
        }

        bool isForward(int degree)
        {
            return (degree >= -45 && degree <= 45) ? true : false;
        };

        bool isForward(Motion motion)
        {
            return ((motion == Motion::GO_FORWARD) ||
                (motion == Motion::CROSS_FORWARD) ||
                (motion == Motion::ENTER_FORWARD) ||
                (motion == Motion::EXIT_FORWARD))
                ? true : false;
        };

        bool isNodeInPath(ID nodeid);
        bool isEdgeInPath(ID edgeid);
        Motion getMotion(int ntype, int etype, int degree);
        Guidance getLastGuidance() { return m_past_guides.back(); };
        std::string getStringAction(Action action);
        std::string getStringFwd(Action act, int ntype, ID nid);
        std::string getStringFwdDist(Action act, int ntype, ID nid, double d);
        std::string getStringFwdDistAfter(Action act, int ntype, ID nid, double d);
        std::string getStringTurn(Action act, int ntype);
        std::string getStringTurnDist(Action act, int ntype, double dist);
        std::string getStringGuidance(Guidance guidance, MovingStatus status);
        int getGuideIdxFromPose(TopometricPose pose);
        MovingStatus getMoveStatus() { return m_mvstatus; };
        LatLon getPoseGPS() { return m_latlon; };

        bool setGuideStatus(TopometricPose pose, double conf);
        bool setGuidanceWithGuideStatus();
        bool setInitialGuide();
        bool setNormalGuide();
        bool setArrivalGuide();
        bool setEmptyGuide();
        //	bool setTunBackGuide();
        bool applyPose(TopometricPose pose);

        //bool setOOPGuide();
        //bool regeneratePath(double start_lat, double start_lon, double dest_lat, double dest_lon);

    public:

        //makeLostValue related variable and method
        double m_prevconf = 1.0;
        double m_lostvalue = 0.0;
        void makeLostValue(double prevconf, double curconf);
    };


}

#endif // End of '__GUIDANCE__'