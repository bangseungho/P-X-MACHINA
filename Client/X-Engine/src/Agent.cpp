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
	float HeuristicManhattan(const Pos& start, const Pos& dest) {
		return static_cast<float>(std::abs(dest.X - start.X) +
			std::abs(dest.Y - start.Y) +
			std::abs(dest.Z - start.Z));
	}
	float HeuristicEuclidean(const Pos& start, const Pos& dest) {
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
	mPathTarget = mObject->GetPosition();
	mMaxNeighbors = 0;
	mNeighborDist = 3.f;
	mTimeHorizon = 2.5f;
	mRadius = 0.2f;
	mMaxSpeed = 3.5f;
	mPrefVelocity = Vec3{};
	mNewVelocity = Vec3{};
	mVelocity = Vec3{};
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
	mVelocity = mNewVelocity;
	mVelocity.y = mNewVelocityY;

	//static const float mdx[4]{ -0.1f, +0.1f, -0.1f, +0.1f };
	//static const float mdz[4]{ -0.1f, -0.1f, +0.1f, +0.1f };
	//static const float odx[4]{ +0.25f, -0.25f, +0.25f, -0.25f };
	//static const float odz[4]{ +0.25f, +0.25f, -0.25f, -0.25f };
	//static const int idx[4]{ +1, -1, +1, -1 };
	//static const int idz[4]{ +1, +1, -1, -1 };

	//const Pos& crntIndex = Scene::I->GetVoxelIndex(mObjectPos);
	//Vec3 nextPos = mObjectPos + mVelocity * DeltaTime();

	//if (abs(mPrevNextPos.y - mObjectPos.y) <= FLT_EPSILON) {
	//	for (int i = 0; i < 4; ++i) {
	//		const Vec3& vertexPos = nextPos + Vec3{ mdx[i], 0.f, mdz[i] };
	//		const Pos& vertexIndex = Scene::I->GetVoxelIndex(vertexPos);

	//		if (Scene::I->CanGoNextVoxel(vertexIndex.Up())) {
	//			continue;
	//		}

	//		const Vec3& obstaclePos = Scene::I->GetVoxelPos(vertexIndex);

	//		int both{};
	//		if (Compare(mObjectPos.x + mdx[i], obstaclePos.x + odx[i], idx[i])) {
	//			mVelocity.x = 0.f;
	//			both++;
	//		}
	//		if (Compare(mObjectPos.z + mdz[i], obstaclePos.z + odz[i], idz[i])) {
	//			mVelocity.z = 0.f;
	//			both++;
	//		}

	//		Pos neighborX = vertexIndex + Pos{ 0, idx[i], 1 };
	//		Pos neighborZ = vertexIndex + Pos{ idz[i], 0, 1 };
	//		if (both == 2) {
	//			if (!Scene::I->CanGoNextVoxel(neighborZ) && Scene::I->CanGoNextVoxel(neighborX)) {
	//				mVelocity.z = mNewVelocity.z;
	//			}
	//			else if (!Scene::I->CanGoNextVoxel(neighborX) && Scene::I->CanGoNextVoxel(neighborZ)) {
	//				mVelocity.x = mNewVelocity.x;
	//			}
	//		}
	//	}

	//	nextPos = mObjectPos + mVelocity * DeltaTime();
	//	for (int i = 0; i < 4; ++i) {
	//		const Vec3& vertexPos = nextPos + Vec3{ mdx[i], 0.f, mdz[i] };
	//		const Pos& vertexIndex = Scene::I->GetVoxelIndex(vertexPos);

	//		if (!Scene::I->CanGoNextVoxel(vertexIndex.Up())) {
	//			mVelocity = -mVelocity;
	//			break;
	//		}
	//	}
	//}

	mObject->SetPosition(mObjectPos + mVelocity * DeltaTime());
}

void Agent::UpdateBegin()
{
	mObjectPos = mObject->GetPosition();
	mVoxelIndex = Scene::I->GetVoxelIndex(mObjectPos);

	//const Vec3& nextPos = mGlobalPath.back() - mObject->GetPosition();
	//const Vec3& crntPathPos = mGlobalPath.back();
	//const Pos& crntPathIndex = Scene::I->GetVoxelIndex(crntPathPos);
	//if (nextPos.Length() <= kMinDistance) {
		//mGlobalPath.pop_back();
		//mGlobalPathCache.erase(crntPathIndex);

	//static Pos prevVoxel{};
	//if (prevVoxel != mVoxelIndex) {
	//	RePlanningToPathAvoidStatic();
	//	prevVoxel = mVoxelIndex;
	//}
	//}
	//else {
		//mVelocity = Vector3::Normalized(nextPos) * mOption.AgentSpeed * DeltaTime();
		//mPrefVelocity = Vector3::Normalized(nextPos);
		//mObject->SetPosition(mObject->GetPosition() + mVelocity);
	//}

	bool isPopPath{};
	while (!mPath.empty()) {
		if (!Scene::I->CanGoNextVoxel(Scene::I->GetVoxelIndex(mPath.back()).Up())) {
			mPath.pop_back();

			if (!mPath.empty()) {
				mPathTarget = mPath.back();
			}

			isPopPath = true;
		}
		else {
			break;
		}
	}

	if (!mPath.empty()) {
		if (mVoxelIndex == Scene::I->GetVoxelIndex(mPathTarget)) {
			mPath.pop_back();

			if (!mPath.empty()) {
				mPathTarget = mPath.back();
			}
		}
	}
	//if (isPopPath) {
	//	//if (Vector3::IsZero(AgentManager::I->GetFlowFieldPos(mVoxelIndex))) {
	//		mStartIndex = mVoxelIndex;
	//		mOption.Heuri = Heuristic::Euclidean;
	//		PathPlanningToAstar(Scene::I->GetVoxelIndex(mPathTarget), {}, false);
	//		std::cout << "NEW PATH!\n";
	//	//}
	//}
}


//const Vec3& Agent::GetCrntPathDir()
//{
//	return GetPathDir(mVoxelIndex);
//}

//const Vec3& Agent::GetPathDir(const Pos& index)
//{
//	if (mGlobalTarget.count(index)) {
//		return mGlobalTarget[index];
//	}
//	return Vector3::Zero;
//}

//const Vec3& Agent::GetCrntPathPos()
//{
//	return GetPathPos(mVoxelIndex);
//}

//const Vec3& Agent::GetPathPos(const Pos& index)
//{
//	if (mGlobalPath.count(index)) {
//		return mGlobalPath[index];
//	}
//	return Vector3::Zero;
//}

//void Agent::SetPathStart(const Vec3& start)
//{
//	mStartPos = mObjectPos;
//	mPrevNextPos = start;
//	mStartIndex = Scene::I->GetVoxelIndex(mStartPos);
//}
//
//void Agent::SetPathDest(const Vec3& dest)
//{
//	mDestPos = dest;
//	mDestIndex = Scene::I->GetVoxelIndex(dest);
//	//mGlobalTarget[mDestIndex] = mDestPos;
//}

std::vector<Vec3> Agent::PathPlanningToAstar(const Pos& dest, const std::unordered_map<Pos, int>& avoidCostMap, bool clearPathList, bool inputDest, int maxOpenNodeCount)
{
	if (clearPathList) {
		ClearPathList();
	}

	std::vector<Vec3> finalPath{};

	mDestIndex = dest;
	std::stack<Pos>	path{};
	std::unordered_map<Pos, Pos>	parent;
	std::unordered_map<Pos, float>	distance;
	std::unordered_map<Pos, bool>	visited;

	std::function<float(const Pos&, const Pos&)> heuristic{};

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

	Pos prevDir;
	PQNode curNode{};
	int openNodeCount{};
	bool failedPlanningPath{};
	while (!pq.empty()) {
		curNode = pq.top();
		prevDir = curNode.Pos - parent[curNode.Pos];
		pq.pop();

		if (openNodeCount >= maxOpenNodeCount) {failedPlanningPath = true; break; }
		if (visited.contains(curNode.Pos)) continue;
		if (distance[curNode.Pos] < curNode.F) continue;
		if (curNode.Pos == dest) break;

		visited[curNode.Pos] = true;

		for (int dir = 0; dir < 8; ++dir) {
			Pos nextPosZX = curNode.Pos + gkFront[dir];
			PairMapRange range = Scene::I->GetCanWalkVoxels(nextPosZX);
			for (auto it = range.first; it != range.second; ++it) {
				Pos nextPos = Pos{ it->first.first, it->first.second, it->second };
				int diffPosY = abs(nextPos.Y - curNode.Pos.Y);
				int avoidCost{}, dirPathCost{};
				int proximityCost = Scene::I->GetProximityCost(nextPos) * PathOption::I->GetProximityWeight();
				//float edgeCost = GetEdgeCost(nextPos, gkFront[dir]) * PathOption::I->GetEdgeWeight();
				if (diffPosY > mOption.ClimbHeight) continue;
				if (visited.contains(nextPos)) continue;
				if (!distance.contains(nextPos)) distance[nextPos] = FLT_MAX;
				if (prevDir != gkFront[dir]) dirPathCost = gkCost[dir];

				auto findIt = avoidCostMap.find(nextPos.XZ());
				if (findIt != avoidCostMap.end()) {
					avoidCost = avoidCostMap.at(nextPos.XZ());
				}

				float g = curNode.G + gkCost[dir] + avoidCost + dirPathCost/* + proximityCost*/ /*+ edgeCost*/;
				float h = (heuristic(nextPos, dest) + mOpenListMinusCost[nextPos] + mPrevPathMinusCost[nextPos]) * PathOption::I->GetHeuristicWeight();

				if (g + h < distance[nextPos]) {
					distance[nextPos] = g + h;
					pq.push({ g + h, g, nextPos });
					parent[nextPos] = curNode.Pos;
					openNodeCount++;
					mOpenList.push_back(nextPos);
				}
			}
		}
	}

	//// 경로 설정 실패
	//if (failedPlanningPath || pq.empty()) {
	//	ClearPathList();
	//	ClearPath();
	//	mDest = AgentManager::I->FindEmptyDestVoxel(this);
	//	return finalPath;
	//}

	Pos pos = curNode.Pos;
	// 부모를 통해 경로 설정
	while (pos != parent[pos]) {
		if (!inputDest) {
			AgentManager::I->mFlowFieldMap[parent[pos]] = Scene::I->GetVoxelPos(pos);
		}
		else {
			mPath.push_back(Scene::I->GetVoxelPos(pos));
			//mGlobalPath[pos] = Scene::I->GetVoxelPos(pos);
			//mGlobalTarget[parent[pos]] = Scene::I->GetVoxelPos(pos);
		}

		pos = parent[pos];
	}

	if (!mPath.empty()) {
		mPathTarget = mPath.back();
	}

	return finalPath;
}

void Agent::ReadyPlanningToPath(const Pos& start)
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

void Agent::RePlanningToPathAvoidStatic()
{
	if (!Vector3::IsZero(AgentManager::I->GetFlowFieldPos(mVoxelIndex))) {
		return;
	}

	//if (!Scene::I->CanGoNextVoxel(mVoxelIndex.Up())) {
	//	mStartIndex = mVoxelIndex;
	//	mOption.Heuri = Heuristic::Euclidean;
	//	PathPlanningToAstar(Scene::I->GetVoxelIndex(mPathTarget), {}, false, false);
	//}

	//Pos nextPathIndex = Scene::I->GetVoxelIndex(GetCrntPathDir());


	//if (!Scene::I->CanGoNextVoxel(nextPathIndex.Up())) {
	//	mStartIndex = mVoxelIndex;
	//	mOption.Heuri = Heuristic::Euclidean;
	//	//mPrevNextPos = mGlobalTarget[nextPathIndex];
	//	//PathPlanningToAstar(Scene::I->GetVoxelIndex(mGlobalTarget[nextPathIndex]), {}, false, false);
	//}
}

void Agent::InsertAgentNeightbor(const Agent* agent, float& rangeSq)
{
	if (this != agent) {
		const float distSq = Vec3::AbsSq(mObject->GetPosition() - agent->GetWorldPosition());

		if (distSq < rangeSq) {
			if (mAgentNeighbors.size() < mMaxNeighbors) {
				mAgentNeighbors.push_back(std::make_pair(distSq, agent));
			}
			
			int i = mAgentNeighbors.size() - 1;
			
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
bool LinearProgram2(const std::vector<Plane>& planes, std::size_t planeNo, float radius, const Vec3& optVelocity, bool directionOpt, Vec3& result) { /* NOLINT(runtime/references) */
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
void LinearProgram4(const std::vector<Plane>& planes, std::size_t beginPlane, float radius, Vec3& result) { 
	/* NOLINT(runtime/references) */
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
		const Vec3 relativePosition = other->GetWorldPosition().xz() - mObject->GetPosition().xz();
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

void Agent::UpdatePrefVelocity(const Vec3& target)
{
	Vec3 toDest{};
	if (Vector3::IsZero(target)) {
		toDest = Vector3::Normalized(mPrevNextPos - mObject->GetPosition()) * mOption.AgentSpeed;
		mNewVelocity = toDest;
		mUseRVO = false;
	}
	else {
		toDest = mDestPos - mObject->GetPosition();
		Vec3 toNext = target - mObject->GetPosition();
		if (Vec3::AbsSq(toDest) > 1.f) {
			toDest = Vector3::Normalized(toNext) * mOption.AgentSpeed;
		}
		mPrefVelocity = toDest;
		mPrevNextPos = target;
		mUseRVO = true;
	}

	mNewVelocityY = toDest.y;
}

void Agent::ClearPath()
{
	mPath.clear();
	mLocalPath.clear();
	mGlobalPathCache.clear();
	mPrevPathMinusCost.clear();
	mOpenListMinusCost.clear();
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

Vec3 AgentManager::GetFlowFieldDirection(const Pos& index)
{
	if (mFlowFieldMap.count(index)) {
		return mFlowFieldMap[index] - Scene::I->GetVoxelPos(index);
	}

	return Vec3{};
}

Vec3 AgentManager::GetFlowFieldPos(const Pos& index)
{
	if (mFlowFieldMap.count(index)) {
		return mFlowFieldMap[index];
	}

	return Vec3{};
}

void AgentManager::AddAgent(Agent* agent)
{
	agent->SetAgentID(++mAgentIDs);
	mAgents.push_back(agent);

	if (!mReader) {
		mReader = mAgents[0];
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
}

void AgentManager::Update()
{
	if (!mReader) {
		return;
	}

	for (auto agent : mAgents) {
		agent->UpdateBegin();

		Vec3 target = agent->GetPathTarget();
		Pos targetIndex = Scene::I->GetVoxelIndex(target);
		int dx = agent->mVoxelIndex.X - targetIndex.X;
		int dz = agent->mVoxelIndex.Z - targetIndex.Z;
		if (std::sqrt(dx * dx + dz * dz) >= 2) {
			agent->RePlanningToPathAvoidStatic();
			target = AgentManager::I->GetFlowFieldPos(agent->mVoxelIndex);
		}

		agent->UpdatePrefVelocity(target);
	}

	mKdTree->BuildAgentTree();

	for (int i = 0; i < static_cast<int>(mAgents.size()); ++i) {
		mAgents[i]->ComputeNeighbors();
		mAgents[i]->ComputeNewVelocity();
	}

	for (auto agent : mAgents) {
		agent->UpdatePosition();
	}
}

void AgentManager::PathPlanningToAstarOnlyReader(const Pos& dest)
{
	AgentManager::I->ClearFlowField();
	mReader->ReadyPlanningToPath(mReader->GetVoxelIndex());
	mReader->PathPlanningToAstar(dest, {}, true);

	const auto& readerPath = mReader->GetPath();
	for (auto agent : mAgents) {
		const Vec3 formation = agent->GetWorldPosition() - mReader->GetWorldPosition();
		
		std::vector<Vec3> path{};
		for (const Vec3& point : readerPath) {
			const Vec3 newPoint = point + formation;
			path.push_back(newPoint);
		}
		
		agent->SetPath(std::move(path));
	}
}

void AgentManager::PathPlanningToFlowField(const Pos& dest)
{
	mIsInit = true;
	std::unordered_map<Pos, float>	distance;
	std::unordered_map<Pos, bool>	visited;

	std::priority_queue<std::pair<float, Pos>, std::vector<std::pair<float, Pos>>, std::greater<std::pair<float, Pos>>> pq;
	pq.push({ 0, dest });
	distance[dest] = 0;

	std::pair<int, Pos> curNode{};
	while (!pq.empty()) {
		curNode = pq.top();
		pq.pop();

		if (distance[curNode.second] < curNode.first)
			continue;

		for (int dir = 0; dir < 8; ++dir) {
			Pos nextPosZX = curNode.second + gkFront[dir];
			PairMapRange range = Scene::I->GetCanWalkVoxels(nextPosZX);
			for (auto it = range.first; it != range.second; ++it) {
				Pos nextPos = Pos{ it->first.first, it->first.second, it->second };
				int diffPosY = abs(nextPos.Y - curNode.second.Y);
				int proximityCost = Scene::I->GetProximityCost(nextPos) * PathOption::I->GetProximityWeight();
				int nextCost = distance[curNode.second] + gkCost[dir] + proximityCost;

				if (diffPosY > mOption.ClimbHeight) continue;
				if (!distance.contains(nextPos)) distance[nextPos] = FLT_MAX;
				if (nextCost >= distance[nextPos]) continue;

				pq.push({ nextCost, nextPos });
				distance[nextPos] = nextCost;
				mFlowFieldMap[nextPos] = Scene::I->GetVoxelPos(curNode.second);
			}
		}
	}
}

void AgentManager::AllAgentPathPlanning(const Pos& dest)
{
	for (auto agent : mAgents) {
		agent->ReadyPlanningToPath(agent->GetVoxelIndex());
		agent->PathPlanningToAstar(dest, {}, true);
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

Pos AgentManager::FindEmptyDestVoxel(Agent* invoker)
{
	std::queue<Pos> q;
	std::map<Pos, bool> visited;
	q.push(invoker->GetPathDest());

	Pos curPos{};
	while (!q.empty()) {
		curPos = q.front();
		q.pop();

		bool isFind = true;
		for (auto agent : mAgents) {
			if (agent == invoker) {
				continue;
			}
			
			if (curPos == agent->GetPathDest()) {
				isFind = false;
				break;
			}
		}

		if (isFind && curPos != invoker->GetPathDest() && curPos != invoker->GetVoxelIndex()) {
			break;
		}

		if (visited[curPos])
			continue;

		visited[curPos] = true;

		for (int dir = 0; dir < 4; ++dir) {
			Pos nextPosZX = curPos + gkFront[dir];
			PairMapRange range = Scene::I->GetCanWalkVoxels(nextPosZX);
			for (auto it = range.first; it != range.second; ++it) {
				Pos nextPos = Pos{ it->first.first, it->first.second, it->second };
				q.push(nextPos);
			}
		}
	}

	return curPos;
}

Pos AgentManager::RandomDest(int x, int z)
{
	const Vec3& cameraTargetPos = MAIN_CAMERA->GetTargetPosition();
	const Pos& cameraTargetIndex = Scene::I->GetVoxelIndex(cameraTargetPos);
	int randZ = Math::RandInt(-x, x);
	int randX = Math::RandInt(-z, z);
	const Vec3& randPos = Scene::I->GetVoxelPos(Pos{ randZ, randX, 0 });
	float y = Scene::I->GetTerrainHeight(randPos.x, randPos.z);
	const Pos& randIndex = Scene::I->GetVoxelIndex(Vec3{ randPos.x, y, randPos.z });
	return cameraTargetIndex.XZ() + randIndex;
}
