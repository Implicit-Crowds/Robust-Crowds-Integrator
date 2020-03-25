#pragma once
#include "AgentInitialParameters.h"
#include "proximitydatabase/Proximity2D.h"
#include <Eigen/Dense>
using namespace Eigen;
typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Vector2D; // do this to avoid alignment issues

class Agent : public ProximityDatabaseItem
{
public:
	Agent();
	~Agent();
	void init(const AgentInitialParameters& initialConditions, SpatialProximityDatabase *const);
	void update(double dt);
	void doStep(double dt);

	/// Overides the isAgent() from ProximityDatabaseItem
	bool isAgent();
	/// Returns true if the agent is active.
	bool isActived() const;
	/// Returns the position of the agent.  
	Vector2D position() const;
	/// Returns the velocity of the agent.  
	Vector2D velocity() const;
	/// Returns the goals of the agent.  
	Vector2D goal() const;
	/// Returns the preferred velocity of the agent.  
	Vector2D vPref() const;
	/// Returns the orientation of the agent.  
	Vector2D orientation() const;
	/// Returns the preferred speed of the agent.  
	double prefSpeed() const;
	/// Returns the radius of the agent.  
	double radius() const;
	/// Returns the id of the agent.  
	int id() const;
	/// Returns the active id of the agent.  
	int activeID() const;
	/// Returns the group id of the agent.  
	int gid() const;
	/// Sets the preferred velocity of the agent to a specific value.	
	void setPreferredVelocity(const Vector2D& v);
	/// Sets the  velocity of the agent to a specific value.	
	void setVelocity(const Vector2D& v);
	/// Sets the active id of the agent to a specific value.	
	void setActiveID(const int& id);
	/// Returns the path of the agent
	vector<Vector2D> path(void) const;
	/// Returns the orientations of the agent across its trajectory
	vector<Vector2D> orientations(void) const;
	/// Finds the neighbors of the agent given a sensing radius
	void findNeighbors(double neighborDist, vector<ProximityDatabaseItem*>& nn);
	//@}

protected:
	inline void destroy();

protected:
	/// The start position of the agent. 
	Vector2D pos;
	/// The goal position of the agent. 
	Vector2D _goal;
	/// The initial velocity of the agent.
	Vector2D vel;
	/// The radius of the agent.
	double r;
	/// The preferred speed of the agent. 
	double prefVel;
	/// The max speed of the agent. 
	double maxVel;
	/// How close to the goal the agent should be to stop the simulation. 
	double goalRadiusSq;
	/// The group id of the agent
	int groupid;
	/// The id of the agent
	int id;
	/// the preferred velocity of the character
	Vector2D _vPref;
	/// Determine whether the charater is enabled;
	bool enabled;
	/// The orientation of the character
	Vector2D _orientation;
	/// The active id of the character. Workaround to account for the fact that the crowd size can dynamically change
	int _activeid;
	/// a pointer to this interface object for the proximity database
	ProximityToken* _proximityToken;
	/// path and orientations
	vector<Vector2D> _path;
	vector<Vector2D> _orientations;
};