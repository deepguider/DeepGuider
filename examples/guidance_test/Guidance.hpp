#pragma once
#include "dg_core.hpp"

namespace dg
{

class Guidance
{
public:
	enum Motion
	{
		FORWARD = 0,
		CROSSWALK,
		CROSSROAD,
		TURN,
	};
	
	template<typename D, typename C>
	class ActionType
	{
	public:
		/**
			* The default constructor
			*/
		ActionType() { }		

		ActionType(D motion, C direction) { this->m_motion = motion; this->m_direction = direction; }

		Motion m_motion;

		int m_direction;

	};

	std::vector<ActionType <Motion, int>> generateGuide (std::vector<NodeInfo>& path);

};
}

