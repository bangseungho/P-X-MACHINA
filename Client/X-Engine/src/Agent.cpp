#include "EnginePch.h"
#include "Component/Agent.h"
#include "Component/Camera.h"
#include "Component/Collider.h"
#include "KdTree.h"

#include "DXGIMgr.h"
#include "Scene.h"
#include "Timer.h"
#include "VoxelManager.h"
#include "InputMgr.h"

namespace {
	float HeuristicManhattan(const Index& start, const Index& dest) {
		return static_cast<float>(std::abs(dest.X - start.X) +
			std::abs(dest.Y - start.Y) +
			std::abs(dest.Z - start.Z));
	}
	float HeuristicEuclidean(const Index& start, const Index& dest) {
		return static_cast<float>(std::sqrt(std::pow(dest.X - start.X, 2) +
			std::pow(dest.Y - start.Y, 2) +
			std::pow(dest.Z - start.Z, 2)));
	}
}

void Agent::ClearPathList()
{
	for (auto& voxel : mCloseList) {
		Scene::I->SetVoxelCondition(voxel, VoxelCondition::None);
	}

	for (auto& voxel : mOpenList) {
		Scene::I->SetVoxelCondition(voxel, VoxelCondition::None);
	}

	mCloseList.clear();
	mOpenList.clear();
}

void Agent::Start()
{
	AgentManager::I->AddAgent(this);
	mCloseList.reserve(10000);
	mOpenList.reserve(10000);
	mPathDir = mObject->GetLook();

	Matrix mtxWorld = mObject->GetWorldTransform();
	mtxWorld = mtxWorld.CreateScale(0.4f);
	SetWorldMatrix(mtxWorld);
	mObject->GetComponent<ObjectCollider>()->SetScale(0.4f);
	mObject->SetPosition(100.f, 0, 260.f);
	mMaxNeighbors = 1;
	mNeighborDist = 3.f;
	mTimeHorizon = 1.5f;
	mRadius = 0.2f;
	mMaxSpeed = 3.5f;
	mPrefVelocity = Vec3{};
	mNewVelocity = Vec3{};
	mVelocity = Vec3{};
}

void Agent::Update()
{
	mObjectPos = mObject->GetPosition();

	if (!Vector3::IsZero(mStartPos)) {
		const Vec3 toDest = mDestPos - GetWorldPosition();
		const Vec3 startToDest = mDestPos - mStartPos;
		const float toDestLength = toDest.Length();
		const float startToDestLength = startToDest.Length();
		mOption.AgentSpeed = std::clamp(3.5f * toDestLength / startToDestLength, 2.5f, 3.5f);
	}
}

bool Compare(float a, float b, int flag)
{
	if (flag == 1) {
		return a > b;
	}
	else if (flag == -1) {
		return a < b;
	}
	return false;
}

void Agent::UpdatePosition()
{
	const Vec3& objectPos = mObjectPos;
	mVoxelIndex = Scene::I->GetVoxelIndex(objectPos);

	if (!PathOption::I->GetStartFlag()) {
		mIsStart = true;
	}

	if (!mIsStart) {
		return;
	}

	mVelocity = mNewVelocity;
	mVelocity.y = mNewVelocityY;

	static const float mdx[4]{ -0.1f, +0.1f, -0.1f, +0.1f };
	static const float mdz[4]{ -0.1f, -0.1f, +0.1f, +0.1f };
	static const float odx[4]{ +0.25f, -0.25f, +0.25f, -0.25f };
	static const float odz[4]{ +0.25f, +0.25f, -0.25f, -0.25f };
	static const int idx[4]{ +1, -1, +1, -1 };
	static const int idz[4]{ +1, +1, -1, -1 };

	const Index& crntIndex = Scene::I->GetVoxelIndex(objectPos);
	Vec3 nextPos = objectPos + mVelocity * DeltaTime();

	if (abs(mPrevNextPos.y - objectPos.y) <= FLT_EPSILON) {
		for (int i = 0; i < 4; ++i) {
			const Vec3& vertexPos = nextPos + Vec3{ mdx[i], 0.f, mdz[i] };
			const Index& vertexIndex = Scene::I->GetVoxelIndex(vertexPos);

			if (Scene::I->CanGoNextVoxel(vertexIndex)) {
				continue;
			}

			const Vec3& obstaclePos = Scene::I->GetVoxelPos(vertexIndex);

			int both{};
			if (Compare(objectPos.x + mdx[i], obstaclePos.x + odx[i], idx[i])) {
				mVelocity.x = 0.f;
				both++;
			}
			if (Compare(objectPos.z + mdz[i], obstaclePos.z + odz[i], idz[i])) {
				mVelocity.z = 0.f;
				both++;
			}

			Index neighborX = vertexIndex + Index{ 0, idx[i], 0 };
			Index neighborZ = vertexIndex + Index{ idz[i], 0, 0 };
			if (both == 2) {
				if (!Scene::I->CanGoNextVoxel(neighborZ) && Scene::I->CanGoNextVoxel(neighborX)) {
					mVelocity.z = mNewVelocity.z;
				}
				else if (!Scene::I->CanGoNextVoxel(neighborX) && Scene::I->CanGoNextVoxel(neighborZ)) {
					mVelocity.x = mNewVelocity.x;
				}
			}
		}

		nextPos = objectPos + mVelocity * DeltaTime();
		for (int i = 0; i < 4; ++i) {
			const Vec3& vertexPos = nextPos + Vec3{ mdx[i], 0.f, mdz[i] };
			const Index& vertexIndex = Scene::I->GetVoxelIndex(vertexPos);

			if (!Scene::I->CanGoNextVoxel(vertexIndex)) {
				mVelocity = -mVelocity;
				break;
			}
		}
	}
	
	mObject->SetPosition(objectPos + mVelocity * DeltaTime());
}

const Index Agent::GetPathIndex(int index) const
{
	if (!mGlobalPath.empty() && mGlobalPath.size() > index) {
		return Scene::I->GetVoxelIndex(mGlobalPath[mGlobalPath.size() - index - 1]);
	}

	return Index{};
}


void Agent::SetStartPos(const Vec3& startPos)
{
	mStartPos = startPos;
	mStartIndex = Scene::I->GetVoxelIndex(startPos);
}

void Agent::SetStartIndex(const Index& startIndex)
{
	mStartIndex = startIndex;
	mStartPos = Scene::I->GetVoxelPos(startIndex);
}

void Agent::SetDestPos(const Vec3& destPos)
{
	mDestPos = destPos;
	mDestIndex = Scene::I->GetVoxelIndex(destPos);
}

void Agent::SetDestIndex(const Index& destIndex)
{
	mDestIndex = destIndex;
	mDestPos = Scene::I->GetVoxelPos(destIndex);
}

std::vector<Vec3> Agent::PathPlanningToAstar(const Index& dest, const std::unordered_map<Index, int>& avoidCostMap, bool followReader, bool clearPathList, bool inputDest, int maxOpenNodeCount)
{
	if (clearPathList) {
		ClearPathList();
	}

	std::vector<Vec3> finalPath{};

	if (dest == mStartIndex) {
		return finalPath;
	}

	SetDestIndex(dest);
	std::stack<Index>	path{};
	std::unordered_map<Index, Index>	parent;
	std::unordered_map<Index, float>	distance;
	std::unordered_map<Index, bool>	visited;

	std::function<float(const Index&, const Index&)> heuristic{};

	switch (mOption.Heuri) {
	case Heuristic::Euclidean:
		heuristic = HeuristicEuclidean;
		break;
	case Heuristic::Manhattan:
		heuristic = HeuristicManhattan;
		break;
	default:
		break;
	}

	// f = g + h
	std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
	float g = 0;
	float h = heuristic(mStartIndex, dest) * PathOption::I->GetHeuristicWeight();
	pq.push({ g + h, g, mStartIndex });
	distance[mStartIndex] = g + h;
	parent[mStartIndex] = mStartIndex;

	Index prevDir;
	PQNode curNode{};
	int openNodeCount{};
	bool failedPlanningPath{};
	while (!pq.empty()) {
		curNode = pq.top();
		prevDir = curNode.Pos - parent[curNode.Pos];
		pq.pop();

		if (openNodeCount >= maxOpenNodeCount) {failedPlanningPath = true; break; }
		if (followReader && mReader->CheckContainNodeInField(curNode.Pos)) break;
		if (visited.contains(curNode.Pos)) continue;
		if (distance[curNode.Pos] < curNode.F) continue;
		if (CheckCurNodeContainPathCache(curNode.Pos)) break;
		if (curNode.Pos == dest) break;

		visited[curNode.Pos] = true;

		for (int dir = 0; dir < 8; ++dir) {
			Index nextPosZX = curNode.Pos + gkFront[dir];
			PairMapRange range = Scene::I->GetCanWalkVoxels(nextPosZX);
			for (auto it = range.first; it != range.second; ++it) {
				Index nextPos = Index{ it->first.first, it->first.second, it->second };
				int diffPosY = abs(nextPos.Y - curNode.Pos.Y);
				int dirPathCost{};
				int proximityCost = Scene::I->GetProximityCost(nextPos) * PathOption::I->GetProximityWeight();
				float edgeCost = GetEdgeCost(nextPos, gkFront[dir]) * PathOption::I->GetEdgeWeight();
				if (diffPosY > mOption.ClimbHeight) continue;
				if (visited.contains(nextPos)) continue;
				if (!distance.contains(nextPos)) distance[nextPos] = FLT_MAX;
				if (prevDir != gkFront[dir]) dirPathCost = gkCost[dir];
				if (followReader) mOpenListMinusCost = mReader->mOpenListMinusCost;
				if (followReader) mPrevPathMinusCost = mReader->mPrevPathMinusCost;
				float g = curNode.G + gkCost[dir] + dirPathCost + proximityCost + edgeCost;
				float h = (heuristic(nextPos, dest) + mOpenListMinusCost[nextPos] + mPrevPathMinusCost[nextPos]) * PathOption::I->GetHeuristicWeight();

				if (g + h < distance[nextPos]) {
					distance[nextPos] = g + h;
					pq.push({ g + h, g, nextPos });
					parent[nextPos] = curNode.Pos;
					openNodeCount++;
				}
			}
		}
	}

	// 경로 설정 실패
	if (failedPlanningPath || pq.empty()) {
		ClearPathList();
		ClearPath();
		mDestIndex = AgentManager::I->FindEmptyDestVoxel(this);
		return finalPath;
	}

	Index pos = curNode.Pos;
	prevDir.Init();

	// 도착점을 넣지 않을 경우
	if (!inputDest) {
		pos = parent[pos];
	}

	// 부모를 통해 경로 설정
	while (pos != parent[pos]) {
		Index dir = parent[pos] - pos;

		if (!PathOption::I->GetDirPathOptimize() || prevDir != dir) {
			path.push(pos);
		}

		mFieldMap.insert({ parent[pos], Vector3::Normalized(Scene::I->GetVoxelPos(pos) - Scene::I->GetVoxelPos(parent[pos])) });
		pos = parent[pos];
		prevDir = dir;
	}

	// 시작점이 적용되지 않을 수 있음
	if (!path.empty() && path.top() != mStartIndex && inputDest) {
		path.push(mStartIndex);
	}

	// 광선을 이용한 경로 최소화
	if (PathOption::I->GetRayPathOptimize()) {
		RayPathOptimize(path, dest);
	}

	// 최종 경로 설정
	std::cout << path.size() << '\n';
	while (!path.empty()) {
		const Index& now = path.top();
		mCloseList.push_back(now);
		mGlobalPathCache.insert({ now, static_cast<int>(finalPath.size()) });
		finalPath.push_back(Scene::I->GetVoxelPos(now));
		path.pop();
	}

	// 경로 캣멀롬 곡선화
	if (PathOption::I->GetSplinePath()) {
		MakeSplinePath(finalPath);
	}

	if (!finalPath.empty()) {
		mPathDir = Vector3::Normalized(finalPath.back() - mObjectPos);
	}

	std::reverse(finalPath.begin(), finalPath.end());

	// 음수 가중치 적용
	for (const Index& openList : mOpenList) {
		if (followReader) {
			mReader->mOpenListMinusCost[openList] = -10;
		}
		else {
			mOpenListMinusCost[openList] = -10;
		}
	}
	for (const Vec3& path : finalPath) {
		if (followReader) {
			mReader->mPrevPathMinusCost[Scene::I->GetVoxelIndex(path)] = -20;
		}
		else {
			mPrevPathMinusCost[Scene::I->GetVoxelIndex(path)] = -20;
		}

	}

	return finalPath;
}

void Agent::ReadyPlanningToPath(const Index& start)
{
	mStartIndex = start;
	mIsStart = false;
	mCloseList.push_back(mStartIndex);
	mNewVelocity = Vec3{};
	mPrefVelocity = Vec3{};
	mVelocity = Vec3{};
	ClearPath();
	mOption.Heuri = Heuristic::Manhattan;
}

#pragma region Optimize
bool Agent::CheckCurNodeContainPathCache(const Index& curNode)
{
	if (mGlobalPathCache.count(curNode)) {
		while (mGlobalPath.size() > 1 && Scene::I->GetVoxelIndex(mGlobalPath.back()) != curNode) {
			mGlobalPathCache.erase(Scene::I->GetVoxelIndex(mGlobalPath.back()));
			mGlobalPath.pop_back();
		}
		return true;
	}
	return false;
}
void Agent::RayPathOptimize(std::stack<Index>& path, const Index& dest)
{
	std::stack<Index> optimizePath{};

	// 현재 시작 지점 설정
	Index now{};
	if (!path.empty()) {
		now = path.top();
		path.pop();
		optimizePath.push(now);
	}

	Index prev = now;
	while (!path.empty()) {
		Index next = path.top();
		Ray ray{};
		ray.Position = Scene::I->GetVoxelPos(now);
		ray.Direction = Scene::I->GetVoxelPos(next) - Scene::I->GetVoxelPos(now);
		ray.Direction.Normalize();

		Index startPoint = Index::Min(now, next);
		Index endPoint = Index::Max(now, next);
		for (int z = startPoint.Z; z <= endPoint.Z; ++z) {
			for (int x = startPoint.X; x <= endPoint.X; ++x) {
				for (int y = startPoint.Y; y <= endPoint.Y; ++y) {
					const Index voxel = Index{ z, x, y };
					VoxelState state = Scene::I->GetVoxelState(voxel); float dist;
					if (ray.Intersects(BoundingBox{ Scene::I->GetVoxelPos(voxel), Grid::mkVoxelExtent }, dist)) {
						if (state == VoxelState::None) {
							optimizePath.push(prev);
							now = prev;
							path.pop();
							goto NoOptimizePath;
						}
						else if (state == VoxelState::Static) {
							if (/*GetOnVoxelCount(voxel) >= mOption.AllowedHeight ||*/ prev.Y != y) {
								optimizePath.push(prev);
								now = prev;
								goto NoOptimizePath;
							}
						}
					}
				}
			}
		}
		path.pop();

	NoOptimizePath:
		prev = next;
	}

	optimizePath.push(dest);

	while (!optimizePath.empty()) {
		path.push(optimizePath.top());
		optimizePath.pop();
	}
}
static Vec3 QuadraticBezier(const Vec3& p0, const Vec3& p1, const Vec3& p2, float t) {
	float u = 1.0f - t;
	return (p0 * (u * u)) + (p1 * (2 * u * t)) + (p2 * (t * t));
}
static Vec3 CubicBezier(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t) {
	float u = 1.0f - t;
	float uu = u * u;
	float uuu = uu * u;
	float tt = t * t;
	float ttt = tt * t;
	return (p0 * uuu) + (p1 * (3 * uu * t)) + (p2 * (3 * u * tt)) + (p3 * ttt);
}
void Agent::MakeSplinePath(std::vector<Vec3>& path)
{
	if (path.size() <= 2) {
		return;
	}

	std::vector<Vec3> splinePath{};
	const int pathSize = static_cast<int>(path.size());
	const int sampleCount = 10;

	Vec3 p4 = path.back() + path[pathSize - 1].xz() - path[pathSize - 2].xz();
	path.push_back(p4);

	for (int i = 0; i < pathSize - 2; ++i) {
		Vec3 p0 = path[i];
		Vec3 p1 = path[i + 1];
		Vec3 p2 = path[i + 2];
		Vec3 p3 = path[i + 3];

		for (int j = 0; j <= sampleCount; ++j) {
			float t = static_cast<float>(j) / sampleCount;
			splinePath.push_back(Vec3::CatmullRom(p0, p1, p2, p3, t));
		}
	}
	path = splinePath;
}
#pragma endregion

void Agent::RePlanningToPathAvoidStatic(const Index& crntPathIndex)
{
	for (int i = mkAvoidForwardStaticObjectCount; i > 0; --i) {
		if (mGlobalPath.size() <= i) {
			continue;
		}

		Index nextPathIndex = Scene::I->GetVoxelIndex(mGlobalPath[mGlobalPath.size() - i]);
		if (!Scene::I->CanGoNextVoxel(nextPathIndex)) {
			for (int j = 0; j < i; ++j) {
				mGlobalPathCache.erase(Scene::I->GetVoxelIndex(mGlobalPath.back()));
				mGlobalPath.pop_back();
			}

			mStartIndex = crntPathIndex;
			mOption.Heuri = Heuristic::Euclidean;
			mLocalPath = PathPlanningToAstar(mDestIndex, {}, true, false, false);
			std::copy(mLocalPath.begin(), mLocalPath.end(), std::back_inserter(mGlobalPath));
			std::cout << "Create Path Count : " << mLocalPath.size() << '\n';
			return;
		}
	}
}

float Agent::GetEdgeCost(const Index& nextPos, const Index& dir)
{
	float cost{};
	if (dir.X != 0 && dir.Z != 0) {
		cost = (Scene::I->GetEdgeCost(nextPos, true) + Scene::I->GetEdgeCost(nextPos, true)) / 2.f;
	}
	else if (dir.Z != 0 && dir.X == 0) {
		cost = Scene::I->GetEdgeCost(nextPos, true);
	}
	else if (dir.X != 0 && dir.Z == 0) {
		cost = Scene::I->GetEdgeCost(nextPos, false);
	}

	return cost;
}

#pragma region RVO
void Agent::InsertAgentNeightbor(const Agent* agent, float& rangeSq)
{
	if (this != agent) {
		const float distSq = Vec3::AbsSq(mObjectPos - agent->GetWorldPosition());

		if (distSq < rangeSq) {
			if (mAgentNeighbors.size() < mMaxNeighbors) {
				mAgentNeighbors.push_back(std::make_pair(distSq, agent));
			}

			int i = static_cast<int>(mAgentNeighbors.size()) - 1;

			while (i != 0 && distSq < mAgentNeighbors[i - 1].first) {
				mAgentNeighbors[i] = mAgentNeighbors[i - 1];
				--i;
			}

			mAgentNeighbors[i] = std::make_pair(distSq, agent);

			if (mAgentNeighbors.size() == mMaxNeighbors) {
				rangeSq = mAgentNeighbors.back().first;
			}
		}
	}
}
void Agent::ComputeNeighbors()
{
	if (!mUseRVO) {
		return;
	}

	mAgentNeighbors.clear();

	if (mMaxNeighbors > 0) {
		AgentManager::I->mKdTree->ComputeAgentNeighbors(this, mNeighborDist * mNeighborDist);
	}
}
bool LinearProgram1(const std::vector<Plane>& planes, std::size_t planeNo, const Line& line, float radius, const Vec3& optVelocity, bool directionOpt, Vec3& result) { /* NOLINT(runtime/references) */
	const float dotProduct = Vec3::Multiply(line.Point, line.Direction);
	const float discriminant = dotProduct * dotProduct + radius * radius - Vec3::AbsSq(line.Point);

	if (discriminant < 0.0F) {
		/* Max speed sphere fully invalidates line. */
		return false;
	}

	const float sqrtDiscriminant = std::sqrt(discriminant);
	float tLeft = -dotProduct - sqrtDiscriminant;
	float tRight = -dotProduct + sqrtDiscriminant;

	for (std::size_t i = 0U; i < planeNo; ++i) {
		const float numerator = Vec3::Multiply((planes[i].Point - line.Point), planes[i].Normal);
		const float denominator = Vec3::Multiply(line.Direction, planes[i].Normal);

		if (denominator * denominator <= FLT_EPSILON) {
			/* Lines line is (almost) parallel to plane i. */
			if (numerator > 0.0F) {
				return false;
			}

			continue;
		}

		const float t = numerator / denominator;

		if (denominator >= 0.0F) {
			/* Plane i bounds line on the left. */
			tLeft = max(tLeft, t);
		}
		else {
			/* Plane i bounds line on the right. */
			tRight = min(tRight, t);
		}

		if (tLeft > tRight) {
			return false;
		}
	}

	if (directionOpt) {
		/* Optimize direction. */
		if (Vec3::Multiply(optVelocity, line.Direction) > 0.0F) {
			/* Take right extreme. */
			result = line.Point + tRight * line.Direction;
		}
		else {
			/* Take left extreme. */
			result = line.Point + tLeft * line.Direction;
		}
	}
	else {
		/* Optimize closest point. */
		const float t = Vec3::Multiply(line.Direction, (optVelocity - line.Point));

		if (t < tLeft) {
			result = line.Point + tLeft * line.Direction;
		}
		else if (t > tRight) {
			result = line.Point + tRight * line.Direction;
		}
		else {
			result = line.Point + t * line.Direction;
		}
	}

	return true;
}
bool LinearProgram2(const std::vector<Plane>& planes, std::size_t planeNo,float radius, const Vec3& optVelocity, bool directionOpt, Vec3& result) { /* NOLINT(runtime/references) */
	const float planeDist = Vec3::Multiply(planes[planeNo].Point, planes[planeNo].Normal);
	const float planeDistSq = planeDist * planeDist;
	const float radiusSq = radius * radius;

	if (planeDistSq > radiusSq) {
		/* Max speed sphere fully invalidates plane planeNo. */
		return false;
	}

	const float planeRadiusSq = radiusSq - planeDistSq;

	const Vec3 planeCenter = planeDist * planes[planeNo].Normal;

	if (directionOpt) {
		/* Project direction optVelocity on plane planeNo. */
		const Vec3 planeOptVelocity =
			optVelocity -
			(optVelocity * planes[planeNo].Normal) * planes[planeNo].Normal;
		const float planeOptVelocityLengthSq = Vec3::AbsSq(planeOptVelocity);

		if (planeOptVelocityLengthSq <= FLT_EPSILON) {
			result = planeCenter;
		}
		else {
			result =
				planeCenter + std::sqrt(planeRadiusSq / planeOptVelocityLengthSq) *
				planeOptVelocity;
		}
	}
	else {
		/* Project point optVelocity on plane planeNo. */
		result = optVelocity +
			((planes[planeNo].Point - optVelocity) * planes[planeNo].Normal) *
			planes[planeNo].Normal;

		/* If outside planeCircle, project on planeCircle. */
		if (Vec3::AbsSq(result) > radiusSq) {
			const Vec3 planeResult = result - planeCenter;
			const float planeResultLengthSq = Vec3::AbsSq(planeResult);
			result = planeCenter +
				std::sqrt(planeRadiusSq / planeResultLengthSq) * planeResult;
		}
	}

	for (std::size_t i = 0U; i < planeNo; ++i) {
		if (Vec3::Multiply(planes[i].Normal, (planes[i].Point - result)) > 0.0F) {
			/* Result does not satisfy constraint i. Compute new optimal result.
			 * Compute intersection line of plane i and plane planeNo.
			 */
			Vec3 crossProduct = planes[i].Normal.Cross(planes[planeNo].Normal);

			if (Vec3::AbsSq(crossProduct) <= FLT_EPSILON) {
				/* Planes planeNo and i are (almost) parallel, and plane i fully
				 * invalidates plane planeNo.
				 */
				return false;
			}

			Line line;
			line.Direction = Vector3::Normalized(crossProduct);
			const Vec3 lineNormal = line.Direction.Cross(planes[planeNo].Normal);
			line.Point = planes[planeNo].Point + (((planes[i].Point - planes[planeNo].Point) * planes[i].Normal) / (lineNormal * planes[i].Normal)) * lineNormal;

			if (!LinearProgram1(planes, i, line, radius, optVelocity, directionOpt,
				result)) {
				return false;
			}
		}
	}

	return true;
}
std::size_t LinearProgram3(const std::vector<Plane>& planes, float radius, const Vec3& optVelocity, bool directionOpt, Vec3& result) { /* NOLINT(runtime/references) */

	if (directionOpt) {
		/* Optimize direction. Note that the optimization velocity is of unit length
		 * in this case.
		 */
		result = optVelocity * radius;
	}
	else if (Vec3::AbsSq(optVelocity) > radius * radius) {
		/* Optimize closest point and outside circle. */
		result = Vector3::Normalized(optVelocity) * radius;
	}
	else {
		/* Optimize closest point and inside circle. */
		result = optVelocity;
	}

	for (std::size_t i = 0U; i < planes.size(); ++i) {
		if (Vec3::Multiply(planes[i].Normal, (planes[i].Point - result)) > 0.0F) {
			/* Result does not satisfy constraint i. Compute new optimal result. */
			const Vec3 tempResult = result;

			if (!LinearProgram2(planes, i, radius, optVelocity, directionOpt,
				result)) {
				result = tempResult;
				return i;
			}
		}
	}

	return planes.size();
}
void LinearProgram4(const std::vector<Plane>& planes, std::size_t beginPlane, float radius, Vec3& result) { /* NOLINT(runtime/references) */
	float distance = 0.0F;

	for (std::size_t i = beginPlane; i < planes.size(); ++i) {
		if (Vec3::Multiply(planes[i].Normal, (planes[i].Point - result)) > distance) {
			/* Result does not satisfy constraint of plane i. */
			std::vector<Plane> projPlanes;

			for (std::size_t j = 0U; j < i; ++j) {
				Plane plane;

				const Vec3 crossProduct = planes[j].Normal.Cross(planes[i].Normal);

				if (Vec3::AbsSq(crossProduct) <= FLT_EPSILON) {
					/* Plane i and plane j are (almost) parallel. */
					if (Vec3::Multiply(planes[i].Normal, planes[j].Normal) > 0.0F) {
						/* Plane i and plane j point in the same direction. */
						continue;
					}

					/* Plane i and plane j point in opposite direction. */
					plane.Point = 0.5F * (planes[i].Point + planes[j].Point);
				}
				else {
					/* Plane.point is point on line of intersection between plane i and
					 * plane j.
					 */
					const Vec3 lineNormal = crossProduct.Cross(planes[i].Normal);
					plane.Point =
						planes[i].Point +
						(((planes[j].Point - planes[i].Point) * planes[j].Normal) /
							(lineNormal * planes[j].Normal)) *
						lineNormal;
				}

				plane.Normal = Vector3::Normalized(planes[j].Normal - planes[i].Normal);
				projPlanes.push_back(plane);
			}

			const Vec3 tempResult = result;

			if (LinearProgram3(projPlanes, radius, planes[i].Normal, true, result) <
				projPlanes.size()) {
				/* This should in principle not happen. The result is by definition
				 * already in the feasible region of this linear program. If it fails,
				 * it is due to small floating point error, and the current result is
				 * kept.
				 */
				result = tempResult;
			}

			distance = Vec3::Multiply(planes[i].Normal, (planes[i].Point - result));
		}
	}
} 
void Agent::ComputeNewVelocity()
{
	if (!mUseRVO) {
		return;
	}

	mORCAPlanes.clear();
	const float invTimeHorizon = 1.f / mTimeHorizon;

	for (int i = 0; i < mAgentNeighbors.size(); ++i) {
		const Agent* const other = mAgentNeighbors[i].second;
		const Vec3 relativePosition = other->GetWorldPosition().xz() - mObjectPos.xz();
		const Vec3 relativeVelocity = mVelocity.xz() - other->GetVelocity().xz();
		const float distSq = Vec3::AbsSq(relativePosition);
		const float combinedRadius = mRadius + other->GetRadius();
		const float combinedRadiusSq = combinedRadius * combinedRadius;

		Plane plane;
		Vec3 u;
		
		if (distSq > combinedRadiusSq) {
			const Vec3 w = relativeVelocity - invTimeHorizon * relativePosition;
			const float wLengthSq = Vec3::AbsSq(w);
			const float dotProduct = Vec3::Multiply(w, relativePosition);

			if (dotProduct < 0.f && dotProduct * dotProduct > combinedRadiusSq * wLengthSq) {
				const float wLength = std::sqrt(wLengthSq);
				const Vec3 unitW = w / wLength;

				plane.Normal = unitW;
				u = (combinedRadius * invTimeHorizon - wLength) * unitW;
			}
			else {
				/* Project on cone. */
				const float a = distSq;
				const float b = Vec3::Multiply(relativePosition, relativeVelocity);
				const float c = Vec3::AbsSq(relativeVelocity) - Vec3::AbsSq(relativePosition.Cross(relativeVelocity)) /(distSq - combinedRadiusSq);
				const float t = (b + std::sqrt(b * b - a * c)) / a;
				const Vec3 ww = relativeVelocity - t * relativePosition;
				const float wwLength = Vec3::Abs(ww);
				const Vec3 unitWW = ww / wwLength;

				plane.Normal = unitWW;
				u = (combinedRadius * t - wwLength) * unitWW;
			}
		}
		else {
			/* Collision. */
			const float invTimeStep = 1.0F / DeltaTime();
			const Vec3 w = relativeVelocity - invTimeStep * relativePosition;
			const float wLength = Vec3::Abs(w);
			const Vec3 unitW = w / wLength;

			plane.Normal = unitW;
			u = (combinedRadius * invTimeStep - wLength) * unitW;
		}

		plane.Point = mVelocity + 0.5f * u;
		mORCAPlanes.push_back(plane);
	}

	const std::size_t planeFail = LinearProgram3(mORCAPlanes, mMaxSpeed, mPrefVelocity.xz(), false, mNewVelocity);
	if (planeFail < mORCAPlanes.size()) {
		LinearProgram4(mORCAPlanes, planeFail, mMaxSpeed, mNewVelocity);
	}
}
#pragma endregion

void Agent::UpdateFollowField()
{
	if (!mReader) {
		return;
	}

	constexpr static float kMaxDistToDest = 10.f;
	if ((mDestPos - mObjectPos).Length() < kMaxDistToDest) {
		return;
	}

	const FieldType nextFieldType = AgentManager::I->GetFieldType(mVoxelIndex);
	const Vec3 toReader = mReader->GetWorldPosition() - mObjectPos;

	constexpr static float kMaxDistToReader = 10.f;
	if (nextFieldType == FieldType::Flower && toReader.Length() > kMaxDistToReader) {
		ReadyPlanningToPath(mVoxelIndex);
		PathPlanningToAstar(mReader->mDestIndex, {}, true);
		AgentManager::I->CopyFlowField(mFieldMap);
		std::cout << "RePlan!\n";
		return;
	}
}

void Agent::SetPreferredVelocity()
{
	Vec3 fieldDirection = AgentManager::I->GetFieldDirection(mVoxelIndex);
	
	if (AgentManager::I->GetFieldType(mVoxelIndex) == FieldType::Reader) {
		const int diffLineCount = mCrntLineCount - AgentManager::I->GetLineCount(mVoxelIndex);
		const float kAngle = 20.f;
		float angleToLine{};
		if (diffLineCount > 0) {
			angleToLine = kAngle;
		}
		else if (diffLineCount < 0) {
			angleToLine = -kAngle;
		}
		fieldDirection = Vector3::Rotate(fieldDirection, Vector3::Up, angleToLine);
	}

	Vec3 toDest{};
	if (Vector3::IsZero(fieldDirection) && AgentManager::I->mIsInit) {
		toDest = (mPrevNextPos - mObjectPos) * mOption.AgentSpeed;
		mNewVelocity = toDest;
		mUseRVO = false;
	}
	else {
		if (!mCrntLineCount) {
			mCrntLineCount = AgentManager::I->GetLineCount(mVoxelIndex);
		}

		toDest = fieldDirection * mOption.AgentSpeed;
		mPrefVelocity = toDest;
		mPrevNextPos = mObjectPos;
		mUseRVO = true;
	}

	mNewVelocityY = toDest.y;
}

void Agent::ClearPath()
{
	mGlobalPath.clear();
	mLocalPath.clear();
	mGlobalPathCache.clear();
	mPrevPathMinusCost.clear();
	mOpenListMinusCost.clear();
	mFieldMap.clear();
}

bool Agent::PickAgent()
{
	const Ray& ray = MAIN_CAMERA->ScreenToWorldRay(InputMgr::I->GetMousePos());

	float distance = 0.f;
	if (ray.Intersects(mObject->GetComponent<ObjectCollider>()->GetBS(), distance)) {
		return true;
	}

	return false;
}

void Agent::RenderOpenList()
{
	for (auto& voxel : mOpenList) {
		if (Scene::I->GetVoxelCondition(voxel) != VoxelCondition::Picked) {
			Scene::I->SetVoxelCondition(voxel, VoxelCondition::Opened);
		}
	}
}

void Agent::RenderCloseList()
{
	for (auto& voxel : mCloseList) {
		if (Scene::I->GetVoxelCondition(voxel) != VoxelCondition::Picked) {
			Scene::I->SetVoxelCondition(voxel, VoxelCondition::Closed);
		}
	}
}

void AgentManager::AddAgent(Agent* agent)
{
	agent->SetAgentID(++mAgentIDs); 
	mAgents.push_back(agent);

	if (!mReader) {
		mReader = agent;
	}
	else {
		agent->SetReader(mReader);
	}
}

void AgentManager::SetClimbHeightAllAgent(int height)
{
	mOption.ClimbHeight = height;
	for (int i = 0; i < static_cast<int>(mAgents.size()); ++i) {
		mAgents[i]->mOption.ClimbHeight = height;
	}
}

void AgentManager::SetAgentSpeedAllAgent(float speed)
{
	mOption.AgentSpeed = speed;
	for (int i = 0; i < static_cast<int>(mAgents.size()); ++i) {
		mAgents[i]->mOption.AgentSpeed = speed;
	}
}

void AgentManager::Start()
{
	mKdTree = std::make_shared<KdTree>();
	PathPlanningToFlowField(Scene::I->GetVoxelIndex({ 100.f, 0, 260.f }));
}

void AgentManager::Update()
{
	for (int i = 0; i < static_cast<int>(mAgents.size()); ++i) {
		//mAgents[i]->UpdateFollowField();
		mAgents[i]->SetPreferredVelocity();
	}

	mKdTree->BuildAgentTree();

	for (int i = 0; i < static_cast<int>(mAgents.size()); ++i) {
		mAgents[i]->ComputeNeighbors();
		mAgents[i]->ComputeNewVelocity();
	}

	for (int i = 0; i < static_cast<int>(mAgents.size()); ++i) {
		mAgents[i]->UpdatePosition();
	}
}

int GetSide2D(const Vec3& A, const Vec3& B, const Vec3& P) {
	Vec3 D = { B.x - A.x, B.y - A.y, B.z - A.z };
	Vec3 W = { P.x - A.x, P.y - A.y, P.z - A.z };
	float cross = D.x * W.z - D.z * W.x;

	if (cross > 0) return 1;   // 왼쪽
	if (cross < 0) return -1;  // 오른쪽
	return 0;  // 직선 위
}

void AgentManager::CopyFlowField(const std::unordered_map<Index, Vec3>& fieldMap)
{
	constexpr static std::array<const Vec3, 4> frontPos{
		Vec3{0.f, 0.f, +Grid::mkVoxelWidth},
		Vec3{+Grid::mkVoxelWidth, 0.f, 0.f},
		Vec3{0.f, 0.f, -Grid::mkVoxelWidth},
		Vec3{-Grid::mkVoxelWidth, 0.f, 0.f},
	};

	constexpr static std::array<const Index, 4> frontIndex{
		Index{+1, 0, 0},
		Index{0, +1, 0},
		Index{-1, 0, 0},
		Index{0, -1, 0},
	};

	std::unordered_map<Index, Vec3> copyMap = fieldMap;
	
	const float kDestLength = 2.f;
	const int kMaxProximity = 3;
	for (int k = 1; k <= mOption.FieldLineCount; ++k) {
		std::vector<std::pair<Index, Vec3>> temp{};
		for (int i = 0; i < static_cast<int>(frontPos.size()); ++i) {
			for (const auto& [index, dir] : fieldMap) {
				const Index nextIndex = index + frontIndex[i] * k;
				const Vec3 nextPos = Scene::I->GetVoxelPos(nextIndex);
				if (Scene::I->GetProximityCost(nextIndex) >= kMaxProximity) continue;
				if (!Scene::I->CanGoNextVoxel(nextIndex)) continue;

				const Vec3 pos = Scene::I->GetVoxelPos(index);
				const Vec3 posDir = pos + dir;
				const int side2D = GetSide2D(pos, posDir, nextPos);

				if (side2D == 0) {
					continue;
				}
				else if (side2D == 1) {
					mLineMap[nextIndex] = -k + mOption.FieldLineCount + 1;
				}
				else{
					mLineMap[nextIndex] = k + mOption.FieldLineCount + 1;
				}

				temp.push_back({ nextIndex, dir });
			}
		}

		for (const auto& v : temp) {
			copyMap.insert(v);
		}
	}

	for (const auto& v : copyMap) {
		mFieldMap[v.first] = v.second;
		mFieldTypeMap[v.first] = FieldType::Reader;
		mReader->mOpenList.push_back(v.first);
	}

	for (const auto& v : fieldMap) {
		mLineMap[v.first] = mOption.FieldLineCount + 1;
		mReader->mFieldMap.insert(v);
	}
}

void AgentManager::PushFlowField(const Index& index, const Vec3& pos)
{
	mFieldMap.insert({ index, pos });
}

void AgentManager::PathPlanningToAStarOnlyReader(const Index& dest)
{
	if (!mReader) {
		return;
	}

	Vec3 totalPos{};
	for (const auto agent : mAgents) {
		totalPos += agent->GetWorldPosition();
	}
	totalPos /= static_cast<float>(mAgents.size());

	mReader->ReadyPlanningToPath(Scene::I->GetVoxelIndex(totalPos));
	mReader->PathPlanningToAstar(dest);
	CopyFlowField(mReader->GetFieldMap());

	for (auto agent : mAgents) {
		agent->SetCrntLineCount(0);
		agent->SetStartPos(agent->GetWorldPosition());
		agent->SetDestPos(mReader->GetDestPos());
	}
}

void AgentManager::PathPlanningToFlowField(const Index& dest)
{
	mIsInit = true;
	std::unordered_map<Index, int>	distance;
	std::unordered_map<Index, bool>	visited;

	std::priority_queue<std::pair<int, Index>, std::vector<std::pair<int, Index>>, std::greater<std::pair<int, Index>>> pq;
	pq.push({ 0, dest });
	distance[dest] = 0;

	std::pair<int, Index> curNode{};
	while (!pq.empty()) {
		curNode = pq.top();
		pq.pop();

		if (distance[curNode.second] < curNode.first) 
			continue;

		for (int dir = 0; dir < 8; ++dir) {
			Index nextPosZX = curNode.second + gkFront[dir];
			PairMapRange range = Scene::I->GetCanWalkVoxels(nextPosZX);
			for (auto it = range.first; it != range.second; ++it) {
				Index nextPos = Index{ it->first.first, it->first.second, it->second };
				int diffPosY = abs(nextPos.Y - curNode.second.Y);
				int proximityCost = Scene::I->GetProximityCost(nextPos) * PathOption::I->GetProximityWeight();
				int nextCost = distance[curNode.second] + gkCost[dir] + proximityCost;

				if (diffPosY > mOption.ClimbHeight) continue;
				if (!distance.contains(nextPos)) distance[nextPos] = INT_MAX;
				if (nextCost >= distance[nextPos]) continue;

				pq.push({ nextCost, nextPos });
				distance[nextPos] = nextCost;

				mFieldMap.insert({ nextPos, Vector3::Normalized(Scene::I->GetVoxelPos(curNode.second) - Scene::I->GetVoxelPos(nextPos)) });
				mFieldTypeMap.insert({ nextPos, FieldType::Flower });
			}
		}
	}
}

void AgentManager::AllAgentPathPlanning(const Index& dest)
{
	for (auto agent : mAgents) {
		agent->ReadyPlanningToPath(agent->GetVoxelIndex());
		agent->PathPlanningToAstar(dest);
	}
}

void AgentManager::StartMoveToPath()
{
	for (Agent* agent : mAgents) {
		agent->SetStartMoveToPath(true);
	}
}

void AgentManager::RenderPathList()
{
	for (Agent* agent : mAgents) {
		agent->RenderOpenList();
	}

	for (Agent* agent : mAgents) {
		agent->RenderCloseList();
	}
}

void AgentManager::ClearPathList()
{
	for (Agent* agent : mAgents) {
		agent->ClearPathList();
	}
}

std::unordered_map<Index, int> AgentManager::CheckAgentIndex(const Index& index, Agent* invoker)
{
	std::unordered_map<Index, int> costMap{};
	for (auto agent : mAgents) {
		if (agent == invoker) {
			continue;
		}
		if (agent->GetPathDirection() == invoker->GetPathDirection() && agent->IsStart()) {
			continue;
		}

		Agent* otherAgent{};
		const Index& agentVoxelIndex = agent->GetVoxelIndex();
		const Index& agentCrntPathIndex = agent->GetPathIndex(0);
		const Index& agentNextPathIndex = agent->GetPathIndex(1);

		if (agentVoxelIndex == index) {
			otherAgent = agent;
		}
		else if (agentCrntPathIndex == index) {
			otherAgent = agent;
		}
		else if (agentNextPathIndex == index) {
			otherAgent = agent;
		}

		if (!otherAgent) {
			continue;
		}

		const Vec3& pos = Scene::I->GetVoxelPos(index);
		for (int z = 1; z >= -1; --z) {
			for (int x = -1; x <= 1; ++x) {
				int dz = index.Z + z;
				int dx = index.X + x;
				const Index& neighborIndex = Index{ dz, dx, 0 };
				const Vec3& neighborPos = Scene::I->GetVoxelPos(neighborIndex);
				const Vec3& dir = Vector3::Normalized(neighborPos.xz() - pos.xz());
				float angle = Vector3::Angle(dir, otherAgent->GetPathDirection());
				int cost = static_cast<int>(pow(1.f - angle / 180.f, 5) * 10);
				costMap[neighborIndex] = max(costMap[neighborIndex], cost);
			}
		}

		// speed clamp : 0.7 ~ 1.4
		float angle = Vector3::Angle(otherAgent->GetPathDirection(), invoker->GetPathDirection());
		float normAngle = max(0.f, (angle / 180.f) * 0.7f) + 0.7f;
		invoker->SetAngleSpeedRatio(normAngle);

		if (agent->GetVoxelIndex() == invoker->GetDestIndex()) {
			invoker->ClearPathList();
			invoker->ClearPath();
			invoker->SetDestIndex(FindEmptyDestVoxel(invoker));
			break;
		}
	}

	if (!costMap.empty()) {
		for (auto agent : mAgents) {
			if (agent == invoker) {
				continue;
			}

			costMap[agent->GetVoxelIndex()] = 1000000;
		}
	}

	return costMap;
}

void AgentManager::PickAgent(Agent** agent)
{
	Agent* pickedAgent{};
	for (Agent* mAgent : mAgents) {
		if (mAgent->PickAgent()) {
			pickedAgent = mAgent;
		}
		mAgent->SetRimFactor(0.f);
	}

	if (pickedAgent) {
		*agent = pickedAgent;
	}

	if (*agent) {
		(*agent)->SetRimFactor(1.f);
	}
}

Index AgentManager::FindEmptyDestVoxel(Agent* invoker)
{
	std::queue<Index> q;
	std::map<Index, bool> visited;
	q.push(invoker->GetDestIndex());

	Index curPos{};
	while (!q.empty()) {
		curPos = q.front();
		q.pop();

		bool isFind = true;
		for (auto agent : mAgents) {
			if (agent == invoker) {
				continue;
			}
			
			if (curPos == agent->GetDestIndex()) {
				isFind = false;
				break;
			}
		}

		if (isFind && curPos != invoker->GetDestIndex() && curPos != invoker->GetVoxelIndex()) {
			break;
		}

		if (visited[curPos])
			continue;

		visited[curPos] = true;

		for (int dir = 0; dir < 4; ++dir) {
			Index nextPosZX = curPos + gkFront[dir];
			PairMapRange range = Scene::I->GetCanWalkVoxels(nextPosZX);
			for (auto it = range.first; it != range.second; ++it) {
				Index nextPos = Index{ it->first.first, it->first.second, it->second };
				q.push(nextPos);
			}
		}
	}

	return curPos;
}
