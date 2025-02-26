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
	Index	Pos{};
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
	float		AgentSpeed = 2.5f;
	int			ClimbHeight = 0;
	int			FieldLineCount = 5;
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

private:
	Agent* mReader{};
	std::unordered_map<Index, Vec3> mFieldMap{};
	INT8 mCrntLineCount{};

	std::vector<Vec3>	mGlobalPath{};
	std::vector<Vec3>	mLocalPath{};
	std::unordered_map<Index, int> mGlobalPathCache{};
	std::unordered_map<Index, int> mOpenListMinusCost{};
	std::unordered_map<Index, int> mPrevPathMinusCost{};

	Vec3				mObjectPos{};
	Index				mVoxelIndex{};

	Index				mStartIndex{};
	Vec3				mStartPos{};
	Index				mDestIndex{};
	Vec3				mDestPos{};

	Vec3				mFormation{};

	Vec3				mPathDir{};

	std::vector<Index>	mCloseList{};
	std::vector<Index>	mOpenList{};
	
	bool				mIsStart{};
	int					mSlowSpeedCount{};
	float				mAngleSpeedRatio{};
	int					mAgentID{};

private:
	static constexpr int mkAvoidForwardStaticObjectCount = 3;

public:
	virtual void Start() override;
	virtual void Update() override;

public:
	void UpdatePosition();
	
public:
	const bool		CheckContainNodeInField(const Index& index) const { return mFieldMap.count(index); }
	const bool		IsStart() const { return mIsStart; }
	const Index		GetPathIndex(int index) const;
	const Index		GetDestIndex() const { return mDestIndex; }
	const Vec3		GetDestPos() const { return mDestPos; }
	const int		GetLineCount() const { return mCrntLineCount; }
	const Matrix	GetWorldMatrix() const { return mObject->GetWorldTransform(); }
	const Vec3		GetWorldPosition() const { return mObject->GetPosition(); }
	Vec3			GetWorldPosition()  { return mObject->GetPosition(); }
	const Vec3		GetPathDirection() const { return mPathDir; }
	const Index		GetVoxelIndex() const { return mVoxelIndex; }
	const std::unordered_map<Index, Vec3>& GetFieldMap() const { return mFieldMap; }

public:
	void SetWorldMatrix(const Matrix& mtxWorld) { return mObject->SetWorldTransform(mtxWorld); }
	void SetStartMoveToPath(bool isStart) { mIsStart = isStart; }
	void SetAngleSpeedRatio(float ratio) { mAngleSpeedRatio = ratio; }
	void SetAgentID(int id) { mAgentID = id; }
	void SetDestIndex(const Index& destIndex);
	void SetDestPos(const Vec3& destPos);
	void SetStartIndex(const Index& startIndex);
	void SetStartPos(const Vec3& startPos);
	void SetReader(Agent* reader) { mReader = reader; }
	void SetCrntLineCount(INT8 count) { mCrntLineCount = count; }
	void SetFormation(const Vec3& formation) { mFormation = formation; }

public:
	std::vector<Vec3>	PathPlanningToAstar(const Index& dest, const std::unordered_map<Index, int>& avoidCostMap = {}, bool followReader = false, bool clearPathList = true, bool inputDest = true, int maxOpenNodeCount = 50000);
	void				ReadyPlanningToPath(const Index& start);
	void				SetPath(std::vector<Vec3>& path) { mGlobalPath = path; }
	bool				PickAgent();
	void				RenderOpenList();
	void				RenderCloseList();
	void				ClearPathList();
	void				ClearPath();
	void				SetRimFactor(float factor) { mObject->mObjectCB.RimFactor = factor; }

private:
	bool	CheckCurNodeContainPathCache(const Index& curNode);
	void	RayPathOptimize(std::stack<Index>& path, const Index& dest);
	void	MakeSplinePath(std::vector<Vec3>& path);

private:
	void	RePlanningToPathAvoidStatic(const Index& crntPathIndex);
	float	GetEdgeCost(const Index& nextPos, const Index& dir);

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
	void UpdateFollowField();
	void UpdateFormation(const Vec3& fieldDirection);
	void UpdatePrefVelocity();
};

enum class FieldType : UINT8 {
	None = 0,
	Reader,
	Flower,
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
	std::unordered_map<Index, Vec3> mFieldMap{};
	std::unordered_map<Index, INT8> mLineMap{};
	std::unordered_map<Index, FieldType> mFieldTypeMap{};

	int mAgentIDs{};
	Agent* mReader{};
	std::vector<Agent*> mAgents{};
	bool mFinishAllAgentMoveToPath{};

public:
	const bool IsFinishAllAgentMoveToPath() const { return mFinishAllAgentMoveToPath; }

public:
	void AddAgent(Agent* agent);

public:
	template<typename T>
	const T GetValueToIndex(const std::unordered_map<Index, T>& map, const Index& index) const;
	const Vec3& GetFieldDirection(const Index& index) const { return GetValueToIndex(mFieldMap, index); };
	FieldType GetFieldType(const Index& index) const { return GetValueToIndex(mFieldTypeMap, index); };
	INT8 GetLineCount(const Index& index) const { return GetValueToIndex(mLineMap, index); };

public:
	void SetAgentPrefVelocity(int agentNo, const Vec3& prefVelocity) { mAgents[agentNo]->mPrefVelocity = prefVelocity; }
	void SetClimbHeightAllAgent(int height);
	void SetAgentSpeedAllAgent(float speed);
	void SetFieldLineCount(int count) { mOption.FieldLineCount = count; }

public:
	void Start();
	void Update();

public:
	void ClearFlowField() { mFieldMap.clear(); mFieldTypeMap.clear(); mLineMap.clear(); }
	void CopyFlowField(const std::unordered_map<Index, Vec3>& fieldMap);
	void PushFlowField(const Index& index, const Vec3& pos);
	void PathPlanningToAStarOnlyReader(const Index& dest);
	void PathPlanningToFlowField(const Index& dest);
	void AllAgentPathPlanning(const Index& dest);
	void StartMoveToPath();
	void RenderPathList();
	void ClearPathList();
	std::unordered_map<Index, int> CheckAgentIndex(const Index& index, Agent* invoker);
	void PickAgent(Agent** agent);
	Index FindEmptyDestVoxel(Agent* agent);
};
#pragma endregion

template<typename T>
inline const T AgentManager::GetValueToIndex(const std::unordered_map<Index, T>& map, const Index& index) const
{
	if (map.count(index)) {
		return map.at(index);
	}

	return T{};
}
