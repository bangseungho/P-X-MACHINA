#pragma once

#pragma region Include
#include "Component/Component.h"
#include <Imgui/ImguiCode/imgui.h>
#pragma endregion


#pragma region Enum
enum class Heuristic : UINT8 {
	Manhattan = 0,
	Euclidean,
};
#pragma endregion


#pragma region Struct
struct PQNode {
	bool operator<(const PQNode& rhs) const { return F < rhs.F; }
	bool operator>(const PQNode& rhs) const { return F > rhs.F; }
	float F{};
	float G{};
	Pos	Pos{};
};
#pragma endregion


#pragma region Class
class PathOption : public Singleton<PathOption> {
	friend Singleton;
	
private:
	int			mMaxOpenNodeCount = 50000;
	int			mOnVoxelCost = 30;
	int			mHeuristicWeight = 10;
	int			mProximityWeight = 10;
	int			mEdgeWeight = 10;
	bool		mDirPathOptimize = false;
	bool		mRayPathOptimize = false;
	bool		mSplinePath = false;
	bool		mStartFlag = false;
	bool		mMoveRandom = false;

public:

	int			GetMaxOpenNodeCount() const { return mMaxOpenNodeCount; }
	int			GetOnVoxelCost() const { return mOnVoxelCost; }
	int			GetHeuristicWeight() const { return mHeuristicWeight; }
	int			GetProximityWeight() const { return mProximityWeight; }
	int			GetEdgeWeight() const { return mEdgeWeight; }
	bool		GetDirPathOptimize() const { return mDirPathOptimize; }
	bool		GetRayPathOptimize() const { return mRayPathOptimize; }
	bool		GetSplinePath() const { return mSplinePath; }
	bool		GetStartFlag() const { return mStartFlag; }
	bool		GetMoveRandom() const { return mMoveRandom; }

	void		SetMaxOpenNodeCount(int count) { mMaxOpenNodeCount = count; }
	void		SetOnVoxelCost(int cost) { mOnVoxelCost = cost; }
	void		SetHeuristicWeight(int weight) { mHeuristicWeight = weight; }
	void		SetProximityWeight(int weight) { mProximityWeight = weight; }
	void		SetEdgeWeight(int weight) { mEdgeWeight = weight; }
	void		SetDirPathOptimize(bool optimize) { mDirPathOptimize = optimize; }
	void		SetRayPathOptimize(bool optimize) { mRayPathOptimize = optimize; if (optimize) SetDirPathOptimize(optimize); }
	void		SetSplinePath(bool spline) { mSplinePath = spline; }
	void		SetStartFlag(bool flag) { mStartFlag = flag; }
	void		SetMoveRandom(bool flag) { mMoveRandom = flag; }
};


struct AgentOption {
	float		AgentSpeed = 1.5f;
	int			ClimbHeight = 0;
	Heuristic	Heuri = Heuristic::Manhattan;
};


struct Plane {
public:
	Vec3 Point{};
	Vec3 Normal{};

public:
	Plane() {}
};

struct Line {
public:
	Vec3 Point{};
	Vec3 Direction{};

public:
	Line() {}
};

class Agent : public Component {
	COMPONENT(Agent, Component)

	friend class AgentManager;
	friend class KdTree;

public:
	AgentOption mOption{};
	bool mUseFlowField{};

private:
	Vec3 mPathTarget{};
	std::vector<Vec3> mPath{};

	//std::unordered_map<Pos, Vec3> mGlobalTarget{};
	//std::unordered_map<Pos, Vec3> mGlobalPath{};

	std::vector<Vec3>	mLocalPath{};
	std::unordered_map<Pos, int> mGlobalPathCache{};
	std::unordered_map<Pos, int> mOpenListMinusCost{};
	std::unordered_map<Pos, int> mPrevPathMinusCost{};

	Vec3				mObjectPos{};
	Pos					mVoxelIndex{};
	Vec3				mStartPos{};
	Pos					mStartIndex{};
	Vec3				mDestPos{};
	Pos					mDestIndex{};
	Vec3				mPathDir{};

	std::vector<Pos>	mCloseList{};
	std::vector<Pos>	mOpenList{};
	
	bool				mIsStart{};
	int					mSlowSpeedCount{};
	float				mAngleSpeedRatio{};
	int					mAgentID{};

	Vec3				mFormationOffset{};
	Vec3				mRelativeVelocity{};

private:
	static constexpr int mkAvoidForwardStaticObjectCount = 3;

public:
	virtual void Start() override;

public:
	void UpdatePosition();
	void UpdateBegin();
	
public:
	const std::vector<Vec3>& GetPath() const { return mPath; }
	const Vec3& GetPathTarget() const { return mPathTarget; }

	const std::unordered_map<Pos, int>& GetPathCache() const { return mGlobalPathCache; }
	const Pos		GetPathDest() const { return mDestIndex; }
	const Vec3&		GetPathStart() const { return mStartPos; }
	const Matrix	GetWorldMatrix() const { return mObject->GetWorldTransform(); }
	const Vec3		GetWorldPosition() const { return mObject->GetPosition(); }
	Vec3			GetWorldPosition()  { return mObject->GetPosition(); }
	const Vec3		GetFormationOffset() const { return mFormationOffset; }
	const Vec3		GetPathDirection() const { return mPathDir; }
	const Pos		GetVoxelIndex() const { return mVoxelIndex; }
	const bool		IsStart() const { return mIsStart; }

public:
	void SetPath(std::vector<Vec3>&& path) { mPath = path; if (!path.empty()) { mPathTarget = mPath.back(); } }

	void SetWorldMatrix(const Matrix& mtxWorld) { return mObject->SetWorldTransform(mtxWorld); }
	void SetStartMoveToPath(bool isStart) { mIsStart = isStart; }
	void SetAngleSpeedRatio(float ratio) { mAngleSpeedRatio = ratio; }
	void SetAgentID(int id) { mAgentID = id; }
	void SetPathDest(const Vec3& dest);
	void SetPathDest(const Pos& dest) { mDestIndex = dest; }
	void SetFormationOffset(const Vec3& offset) { mFormationOffset = offset; }

public:
	std::vector<Vec3>	PathPlanningToAstar(const Pos& dest, const std::unordered_map<Pos, int>& avoidCostMap, bool clearPathList = true, bool inputDest = true, int maxOpenNodeCount = 50000);
	void				ReadyPlanningToPath(const Pos& start);
	void				SetPathCache(std::unordered_map<Pos, int> cache) { mGlobalPathCache = cache; }
	bool				PickAgent();
	void				RenderOpenList();
	void				RenderCloseList();
	void				ClearPathList();
	void				ClearPath();
	void				SetRimFactor(float factor) { mObject->mObjectCB.RimFactor = factor; }

private:
	void				RePlanningToPathAvoidStatic();

private:
	std::vector<std::pair<float, const Agent*>> mAgentNeighbors{};
	std::vector<Plane> mORCAPlanes{};
	int mMaxNeighbors{};
	float mNeighborDist{};
	float mTimeHorizon{};
	float mRadius{};
	float mMaxSpeed{};

	Vec3 mVelocity{};
	Vec3 mNewVelocity{};
	Vec3 mPrefVelocity{};
	Vec3 mPrevNextPos{};
	float mNewVelocityY{};
	bool mUseRVO{};

public:
	Vec3 GetVelocity() const { return mVelocity; }
	float GetRadius() const { return mRadius; }

public:
	void InsertAgentNeightbor(const Agent* agent, float& rangeSq);
	void ComputeNeighbors();
	void ComputeNewVelocity();
	void UpdatePrefVelocity(const Vec3& target);
};


class AgentManager : public Singleton<AgentManager> {
	friend Singleton;
	friend class KdTree;
	friend class Agent;

public:
	bool mIsInit{};
	AgentOption mOption{};

private:
	sptr<class KdTree> mKdTree{};
	std::unordered_map<Pos, Vec3> mFlowFieldMap{};

	int mAgentIDs{};
	Agent* mReader{};
	std::vector<Agent*> mAgents{};
	bool mFinishAllAgentMoveToPath{};

public:
	const bool IsFinishAllAgentMoveToPath() const { return mFinishAllAgentMoveToPath; }

public:
	void AddAgent(Agent* agent);
	void SetAgentPrefVelocity(int agentNo, const Vec3& prefVelocity) { mAgents[agentNo]->mPrefVelocity = prefVelocity; }
	void EraseFlowFieldPos(const Pos& pos) { mFlowFieldMap.erase(pos); }
	Vec3 GetFlowFieldDirection(const Pos& pos);
	Vec3 GetFlowFieldPos(const Pos& pos);

public:
	void SetClimbHeightAllAgent(int height);
	void SetAgentSpeedAllAgent(float speed);

public:
	void Start();
	void Update();

public:
	void PathPlanningToFlowField(const Pos& dest);
	void PathPlanningToAstarOnlyReader(const Pos& dest);
	void AllAgentPathPlanning(const Pos& dest);
	void StartMoveToPath();
	void RenderPathList();
	void ClearPathList();
	void PickAgent(Agent** agent);
	Pos FindEmptyDestVoxel(Agent* agent);
	void ClearFlowField() { mFlowFieldMap.clear(); }

public:
	Pos RandomDest(int x, int z);
};

#pragma endregion