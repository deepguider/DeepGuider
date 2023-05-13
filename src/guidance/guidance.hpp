#ifndef __GUIDANCE__
#define __GUIDANCE__

#include "dg_core.hpp"
#include "core/shared_data.hpp"

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

        enum class RobotStatus
        {
            READY = 0,
            RUN_MANUAL,
            RUN_AUTO,
            ARRIVED_NODE,
            ARRIVED_GOAL,
            NO_PATH,

            TYPE_NUM
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
            int node_type = 0;
            int edge_type = 0;	//
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
            GuideStatus guide_status = GuideStatus::GUIDE_NOPATH;	// current guidance status
            std::vector<Action> actions;// action command
            ID heading_node_id = 0;	        // heading node
            double relative_angle = 0.0;
            double distance_to_remain = 0;  // distance to heading node (unit: meter)
            std::string msg = "";		    // string guidance message
            bool announce = false;
        };

    // protected:
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
            int cur_degree = 0;

            /** junction of guidance */
            bool is_junction = false;
            double remain_distance_to_next_junction = 0;
            double past_distance_from_prev_junction = 0;
            ID next_guide_node_id = 0;
            ID next_guide_edge_id = 0;
            int next_junction_idx = 0;
            int junction_degree = 0;
        };

    public:
        GuidanceManager() { }

        bool m_dxrobot_usage = 0;
        int m_robot_map_idx = -1;
        ID m_robot_heading_node_id;
        bool m_use_online_map = 0;
        cv::Mutex m_robot_mutex;
        dg::Pose2 m_robot_pose;
        dg::Pose2 m_dg_pose;
        dg::Pose2 m_prev_dg_pose;
        dg::Pose2 m_temp_prev_dg_pose;
        dg::Pose2 m_temp2_prev_dg_pose;
        dg::Pose2 m_prev_dx_pose;
        dg::Pose2 m_temp_prev_dx_pose;
        dg::Pose2 m_temp2_prev_dx_pose;
        cv::Point2d m_robot_on_image;
        cv::Point2d m_subgoal_pose;
        cv::Point2d m_robot_heading_node_pose;
        bool initialize(dg::SharedInterface* shared);
        bool initiateNewGuidance();
        bool initiateNewGuidance(Point2F gps_start, Point2F gps_des);
        bool initiateNewGuidance(TopometricPose pose_topo, Point2F gps_des);
        bool update(TopometricPose pose);
        bool update(TopometricPose pose, Pose2 pose_metric);
        bool updateWithRobot(TopometricPose pose, Pose2 pose_metric);
        GuideStatus getGuidanceStatus() const { return m_gstatus; };
        Guidance getGuidance() const { return m_curguidance; };
        RobotStatus getRobotStatus(){ return m_robot_status; }; 
        bool isGuidanceInitialized() {
            if (m_guide_idx < 0) return false;
            else return true;        }
        bool isExpendedPathGenerated(){
            if (m_extendedPath.size() > 0) return true;
            else return false;}
        ExtendedPathElement getPrevExtendedPath() { return m_extendedPath[std::max(m_guide_idx-1, 0)]; };
        ExtendedPathElement getCurExtendedPath() { return m_extendedPath[std::max(m_guide_idx, 0)]; };
        ExtendedPathElement getNextExtendedPath() { 
            return m_extendedPath[std::min(m_guide_idx+1, (int) m_extendedPath.size()-1)]; };
        ExtendedPathElement getNextNextExtendedPath() { 
            return m_extendedPath[std::min(m_guide_idx+2, (int) m_extendedPath.size()-1)]; };
            
        int getCurGuideIdx() { return m_guide_idx; };
        int getLastGuideIdx() { return (int)m_extendedPath.size()-1; };

        void setRobotStatus(RobotStatus status) 
        { 
            m_robot_status = status; 
        };
        void setRobotUsage(cv::String name) 
        { 
            cv::String robot = "KETI_ROBOT";
	        if (name.compare(robot)==0) m_dxrobot_usage = true;
            else m_dxrobot_usage = false; 
        };
        //from dg_ros.yml: 0:"Bucheon_KETI", 1:"Bucheon_Robot", 2:"COEX_KETI", 3:"COEX_KETI_220824"
        void setRobotMap(cv::String name)
        {
            if (name.compare("Bucheon_KETI")==0)
                m_robot_map_idx = 0;
            else if (name.compare("Bucheon_Robot")==0)
                m_robot_map_idx = 1;
            else if (name.compare("COEX_KETI")==0)
                m_robot_map_idx = 2;
            else if (name.compare("COEX_KETI_220824")==0)
                m_robot_map_idx = 3;
            else
            {
                m_robot_map_idx = -1;
            }
            
        };
        void setRobotMapOnOff(bool flag){
            if(flag) m_use_online_map = true;
            else m_use_online_map = false;
        };
        int getDegree(Point2 p1, Point2 p2, Point2 p3);
        std::vector <ExtendedPathElement> m_extendedPath;

        double m_uncertain_dist_public() const { return m_uncertain_dist; }  // for public readonly access

    protected:
        SharedInterface* m_shared = nullptr;
        Map* getMap() { if (m_shared) return m_shared->getMap(); return nullptr; }

        bool validatePath(const Path& path, const Map& map);
        bool buildGuides();

    private:
        // std::vector <ExtendedPathElement> m_extendedPath;
        GuideStatus  m_gstatus = GuideStatus::GUIDE_NORMAL;
        std::vector<Guidance> m_past_guides;
        Guidance m_curguidance;
        RobotStatus m_robot_status = RobotStatus::READY;
        int m_guide_idx = -1;	//starts with -1 because its pointing current guide.
        int m_robot_guide_idx = -1;
        double m_remain_distance = 0.0;
        int m_last_announce_dist = -1;
        bool m_arrival = false;
        int m_arrival_cnt = 0;

        //fixed parameter
        int m_guide_interval = 10; //m
        double m_uncertain_dist = 5.0;
        double m_arrived_threshold = 2.0;
        double m_start_exploration_dist = 5.0;
        int m_max_arrival_cnt = 10;

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

        int getGuideIdxFromPose(TopometricPose pose);
        ExtendedPathElement getExtendedPath(int idx);
        ExtendedPathElement getLastExtendedPath(){ return m_extendedPath.back(); };

        std::string getStringAction(Action action);
        std::string getStringFwd(Action act, int ntype, ID nid);
        std::string getStringFwdDist(Action act, int ntype, ID nid, double d);
        std::string getStringTurn(Action act, int ntype);
        std::string getStringTurnDist(Action act, int ntype, double dist);
        std::string getStringGuidance(Guidance guidance);

        Motion getMotion(int ntype, int etype, int degree);
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
        };

        bool setSimpleGuide();
        bool setArrivalGuide();
        bool setEmptyGuide();
        Action setActionTurn(ID nid_cur, ID eid_cur, int degree);
        Action setActionGo(ID nid_next, ID eid_cur, int degree = 0);
        bool setHeadingPoint();

        bool isRobotNearArrivalNode();
        bool isDGNearArrivalNode();
        bool isNodeInPath(ID nodeid);
        bool isEdgeInPath(ID edgeid);
        bool isNodeInPastGuides(ID nodeid);
        bool isForward(int degree)
        {
            return (degree >= -30 && degree <= 30) ? true : false;
        };
        bool isForward(Motion motion)
        {
            return ((motion == Motion::GO_FORWARD) ||
                (motion == Motion::CROSS_FORWARD) ||
                (motion == Motion::ENTER_FORWARD) ||
                (motion == Motion::EXIT_FORWARD))
                ? true : false;
        };
               
    public:

        //makeLostValue related variable and method
        double m_prevconf = 1.0;
        double m_lostvalue = 0.0;
        void makeLostValue(double prevconf, double curconf);
    };

}

#endif // End of '__GUIDANCE__'