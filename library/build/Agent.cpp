#include "Agent.h"


Agent::Agent()
{
	enabled = false;
	_proximityToken = NULL;
}

Agent::~Agent()
{
	destroy();
}

void Agent::destroy()
{
	if (_proximityToken != NULL)
	{
		delete _proximityToken;
		_proximityToken = 0x0;
	}
}

void Agent::init(const AgentInitialParameters& initialConditions, SpatialProximityDatabase *const pd)
{
	// initialize the agent based on the initial conditions
	pos = initialConditions.position;
	r = initialConditions.radius;
	prefVel = initialConditions.prefSpeed;
	_id = initialConditions.id;
	_activeid = _id;
	groupid = initialConditions.gid;
	goalRadiusSq = initialConditions.goalRadius*initialConditions.goalRadius;
	vel = initialConditions.velocity;
	_goal = initialConditions.goal;
	_orientation = (_goal - pos).normalized();
	enabled = true;

	//add to the database
	_proximityToken = pd->allocateToken(this);
	// notify proximity database that our position has changed
	_proximityToken->updateForNewPosition(pos);

	// add initial position, orientation
	_path.push_back(position());
	_orientations.push_back(_orientation);
}


void Agent::doStep(double dt)
{
	_vPref = _goal - pos;
	double distSqToGoal = _vPref.squaredNorm();
	if (distSqToGoal < goalRadiusSq)
	{
		destroy();
		enabled = false;
		return;
	}

	// compute preferred velocity
	if (prefVel * dt*prefVel * dt > distSqToGoal)
		_vPref = _vPref / dt;
	else
		_vPref *= prefVel / sqrt(distSqToGoal);
}



void Agent::update(double dt)
{
	//clamp(velocity, maxSpeed);		
	pos += vel * dt;

	//simple smoothing of the orientation; there are more elaborate approaches
	if (vel.x() != 0 || vel.y() != 0)
		_orientation = _orientation + (vel.normalized() - _orientation) * 0.4;

	// notify proximity database that our position has changed
	_proximityToken->updateForNewPosition(pos);
	// add position and orientation to the list
	_path.push_back(position());
	_orientations.push_back(_orientation);
}

void Agent::findNeighbors(double neighborDist, vector<ProximityDatabaseItem*>& nn)
{
	_proximityToken->findNeighbors(pos, neighborDist, nn);
}

bool Agent::isAgent() {
	return true;
}
/// Returns true if the agent is active.
bool Agent::isActived() const {
	return enabled;
}
/// Returns the position of the agent.  
Vector2D Agent::position() const {
	return pos;
}
/// Returns the velocity of the agent.  
Vector2D Agent::velocity() const {
	return vel;
}
/// Returns the goals of the agent.  
Vector2D Agent::goal() const {
	return _goal;
}
/// Returns the preferred velocity of the agent.  
Vector2D Agent::vPref() const {
	return _vPref;
}
/// Returns the orientation of the agent.  
Vector2D Agent::orientation() const {
	return _orientation;
}
/// Returns the preferred speed of the agent.  
double Agent::prefSpeed() const {
	return prefVel;
}
/// Returns the radius of the agent.  
double Agent::radius() const {
	return r;
}
/// Returns the id of the agent.  
int Agent::id() const {
	return _id;
}
/// Returns the active id of the agent.  
int Agent::activeID() const {
	return _activeid;
}
/// Returns the group id of the agent.  
int Agent::gid() const {
	return groupid;
}
/// Sets the preferred velocity of the agent to a specific value.	
void Agent::setPreferredVelocity(const Vector2D& v) {
	_vPref = v;
}
/// Sets the  velocity of the agent to a specific value.	
void Agent::setVelocity(const Vector2D& v) {
	vel = v;
}
/// Sets the active id of the agent to a specific value.	
void Agent::setActiveID(const int& id) {
	_activeid = id;
}
/// Returns the path of the agent
vector<Vector2D> Agent::path(void) const {
	return _path;
}
/// Returns the orientations of the agent across its trajectory
vector<Vector2D> Agent::orientations(void) const {
	return _orientations;
}