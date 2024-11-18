#include "Collisions.h"

bool Collisions::checkBoundingBox(
	Circle& circle,
	rectBarrier& barrier
)
{
	//NOTE: ASSUMES BARRIER IS NOT ROTATED

	float maxX_circle = 0.f;
	float minX_circle = 0.f;
	float maxY_circle = 0.f;
	float minY_circle = 0.f;

	sf::Vector2f barrierSize = barrier.size;

	float maxX_barrier = barrier.position.x + barrierSize.x;
	float minX_barrier = barrier.position.x - barrierSize.x;
	float maxY_barrier = barrier.position.y + barrierSize.y;
	float minY_barrier = barrier.position.y - barrierSize.y;

	circle.getBoundingBox(maxX_circle, minX_circle, maxY_circle, minY_circle);

	return (minX_circle < maxX_barrier && maxX_circle > minX_barrier &&
		minY_circle < maxY_barrier && maxY_circle > minY_barrier);
}

bool Collisions::checkBoundingBox(
	ConvexPolygon& polygon,
	rectBarrier& barrier
)
{
	//NOTE: ASSUMES BARRIER IS NOT ROTATED

	float maxX_polygon = 0.f;
	float minX_polygon = 0.f;
	float maxY_polygon = 0.f;
	float minY_polygon = 0.f;

	sf::Vector2f barrierSize = barrier.size;

	float maxX_barrier = barrier.position.x + barrierSize.x;
	float minX_barrier = barrier.position.x - barrierSize.x;
	float maxY_barrier = barrier.position.y + barrierSize.y;
	float minY_barrier = barrier.position.y - barrierSize.y;

	polygon.getBoundingBox(maxX_polygon, minX_polygon, maxY_polygon, minY_polygon);

	return (minX_polygon < maxX_barrier && maxX_polygon > minX_barrier &&
		minY_polygon < maxY_barrier && maxY_polygon > minY_barrier);
}

bool Collisions::checkBoundingBox(
	Circle& circle,
	ConvexPolygon& polygon
)
{
	float maxX_circle = 0.f;
	float minX_circle = 0.f;
	float maxY_circle = 0.f;
	float minY_circle = 0.f;

	float maxX_polygon = 0.f;
	float minX_polygon = 0.f;
	float maxY_polygon = 0.f;
	float minY_polygon = 0.f;

	circle.getBoundingBox(maxX_circle, minX_circle, maxY_circle, minY_circle);
	polygon.getBoundingBox(maxX_polygon, minX_polygon, maxY_polygon, minY_polygon);

	return (minX_circle < maxX_polygon && maxX_circle > minX_polygon &&
		minY_circle < maxY_polygon && maxY_circle > minY_polygon);
}

bool Collisions::checkBoundingBox(
	ConvexPolygon& polygonA,
	ConvexPolygon& polygonB
)
{
	float maxX_A = 0.f;
	float minX_A = 0.f;
	float maxY_A = 0.f;
	float minY_A = 0.f;

	float maxX_B = 0.f;
	float minX_B = 0.f;
	float maxY_B = 0.f;
	float minY_B = 0.f;

	polygonA.getBoundingBox(maxX_A, minX_A, maxY_A, minY_A);
	polygonB.getBoundingBox(maxX_B, minX_B, maxY_B, minY_B);

	return (minX_A < maxX_B && maxX_A > minX_B &&
		minY_A < maxY_B && maxY_A > minY_B);
}

void Collisions::circleCircleDetection(
	Circle& c1,
	Circle& c2
)
{
	sf::Vector2f separation = c2.currentPosition - c1.currentPosition;
	float separation_magnitude = pow((separation.x * separation.x) + (separation.y * separation.y), 0.5f);

	if (separation_magnitude < c1.size + c2.size) {
		circleCircleResolution(c1, c2, separation_magnitude, (separation / separation_magnitude));
	}
}

void Collisions::circleCircleResolution(
	Circle& c1,
	Circle& c2,
	float depth,
	sf::Vector2f axis
)
{
	sf::Vector2f overLapCorrection = 0.5f * (c1.size + c2.size - depth) * axis;

	c1.currentPosition -= overLapCorrection;
	c1.oldPosition -= overLapCorrection;

	c2.currentPosition += overLapCorrection;
	c2.oldPosition += overLapCorrection;

	//Resolve collision between the circles
	const sf::Vector2f vel_c2 = c2.currentPosition - c2.oldPosition;
	const sf::Vector2f vel_c1 = c1.currentPosition - c1.oldPosition;

	const float norm_vel_relative = VectorMath::dotProduct(vel_c2 - vel_c1, axis);

	//Only take impulse into account if objects are moving towards each other.
	if (norm_vel_relative > 0.f) {
		return;
	}

	float restitutionCoefficient = (c1.resCoeff > c2.resCoeff ? c2.resCoeff : c1.resCoeff);

	float j = -(1.f + restitutionCoefficient) *
		norm_vel_relative / ((1.f / c1.mass) + (1.f / c2.mass));


	c1.oldPosition += (j / (1.f / c1.mass)) * axis;
	c2.oldPosition -= (j / (1.f / c2.mass)) * axis;
}

void Collisions::circlePolygonDetection(
	Circle& c,
	ConvexPolygon& polygon
)
{
	if (!Collisions::checkBoundingBox(c, polygon)) {
		return;
	}

	sf::Vector2f closestPoint;
	int polygonVertexCount = polygon.body.getPointCount();

	for (int i = 0; i < polygonVertexCount; ++i) {
		sf::Vector2f tempVector = VectorMath::closestPointOnLineSegment(c.currentPosition, polygon.getVertexPosition(i),
			polygon.getVertexPosition((i + 1) % polygonVertexCount));
		float tempDistance = VectorMath::findDistance(c.currentPosition, tempVector);

		if (tempDistance < c.size) {
			sf::Vector2f axis = tempVector - c.currentPosition;
			axis = VectorMath::normalise(axis);

			//Check the normal is facing in the correct direction
			if (VectorMath::dotProduct(polygon.currentPosition - c.currentPosition, axis) < 0.f) {
				axis = { -axis.x, -axis.y };
			}
			Collisions::circlePolygonResolution(c, polygon, (c.size - tempDistance), axis, tempVector - polygon.currentPosition);
		}
	}
}

void Collisions::circlePolygonResolution(
	Circle& c,
	ConvexPolygon& polygon,
	float depth,
	sf::Vector2f axis,
	const sf::Vector2f& contactPointOnPolygon
)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.
	//Denoting circle as A, polygon as B and contact point as P.
	sf::Vector2f overlapCorrection = 0.5f * depth * axis;

	c.currentPosition -= overlapCorrection;
	c.oldPosition -= overlapCorrection;

	polygon.currentPosition += overlapCorrection;
	polygon.oldPosition += overlapCorrection;

	sf::Vector2f r_PA = (polygon.currentPosition + contactPointOnPolygon) - c.currentPosition;

	sf::Vector2f v_A = c.currentPosition - c.oldPosition;
	v_A -= (c.currentAngle - c.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(r_PA);

	sf::Vector2f v_B = polygon.currentPosition - polygon.oldPosition;
	v_B -= (polygon.currentAngle - polygon.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(contactPointOnPolygon);

	float contactVelocity_mag = VectorMath::dotProduct(v_B - v_A, axis);

	if (contactVelocity_mag > 0.f) {
		return;
	}

	float r_PA_PerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(r_PA), axis);
	float r_PB_PerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(contactPointOnPolygon), axis);

	float denom = (1.f / c.mass) + (1.f / polygon.mass) +
		(r_PA_PerpDotN * r_PA_PerpDotN) * (1.f / c.momentOfInertia) +
		(r_PB_PerpDotN * r_PB_PerpDotN) * (1.f / polygon.momentOfInertia);

	//Modify this to find the lower of the two restitution coefficients
	float restitutionCoefficient = (c.resCoeff > polygon.resCoeff ? polygon.resCoeff : c.resCoeff);

	float j = -(1.f + restitutionCoefficient) * contactVelocity_mag;
	j /= denom;

	sf::Vector2f impulse = j * axis;

	c.oldPosition += impulse * (1.f / c.mass);
	c.oldAngle += VectorMath::crossProduct(r_PA, impulse) * (1.f / c.momentOfInertia);

	polygon.oldPosition -= impulse * (1.f / polygon.mass);
	polygon.oldAngle -= VectorMath::crossProduct(contactPointOnPolygon, impulse) * (1.f / polygon.momentOfInertia);
}

void Collisions::polygonPolygonDetection(
	ConvexPolygon& polygonA,
	ConvexPolygon& polygonB
)
{
	if (!Collisions::checkBoundingBox(polygonA, polygonB)) {
		return;
	}

	float minimumDepth = VectorMath::upperLimit;
	sf::Vector2f minimumAxis = { -1.f, -1.f };

	std::vector<sf::Vector2f> polygonAVertices = polygonA.getAllVertices();
	std::vector<sf::Vector2f> polygonBVertices = polygonB.getAllVertices();

	if ((projectionSAT(polygonAVertices, polygonBVertices, minimumDepth, minimumAxis)) ||
		(projectionSAT(polygonBVertices, polygonAVertices, minimumDepth, minimumAxis))) {
		return;
	}

	//Polygons are colliding and the minimum distance in order to separate the two has been found.

	//Check the normal is facing in the correct direction
	if (VectorMath::dotProduct(polygonB.currentPosition - polygonA.currentPosition, minimumAxis) < 0.f) {
		minimumAxis = -minimumAxis;
	}

	polygonPolygonResolution(polygonA, polygonB, minimumDepth, minimumAxis);
}

bool Collisions::projectionSAT(
	std::vector<sf::Vector2f>& polygonAVertices,
	std::vector<sf::Vector2f>& polygonBVertices,
	float& minDepth,
	sf::Vector2f& minAxis
)
{
	//SAT implementation to find out whether two polygons have collided
	//Similar to circle-polygon collision detection

	int polygonAVertexCount = polygonAVertices.size();
	int polygonBVertexCount = polygonBVertices.size();

	for (int i = 0; i < polygonAVertexCount; ++i) {

		sf::Vector2f axis;
		float depth;

		sf::Vector2f vertex1Position = polygonAVertices[i];
		sf::Vector2f vertex2Position = polygonAVertices[(i + 1) % polygonAVertexCount];

		axis = VectorMath::perpendicular(vertex2Position - vertex1Position);
		axis = VectorMath::normalise(axis);

		//Project all the vertices of both polygons onto the axis. Find the max and min for both polygons
		float max1 = VectorMath::lowerLimit;
		float min1 = VectorMath::upperLimit;

		float max2 = VectorMath::lowerLimit;
		float min2 = VectorMath::upperLimit;

		for (int j = 0; j < polygonAVertexCount; ++j) {
			sf::Vector2f vertexPosition = polygonAVertices[j];
			float projection = VectorMath::dotProduct(vertexPosition, axis);

			if (projection > max1) {
				max1 = projection;
			}
			if (projection < min1) {
				min1 = projection;
			}
		}
		for (int j = 0; j < polygonBVertexCount; ++j) {
			sf::Vector2f vertexPosition = polygonBVertices[j];
			float projection = VectorMath::dotProduct(vertexPosition, axis);

			if (projection > max2) {
				max2 = projection;
			}
			if (projection < min2) {
				min2 = projection;
			}
		}

		if (max1 < min2 || max2 < min1) {
			//Not colliding.
			return true;
		}
		//Projections are overlapping
		if (max1 - min2 < max2 - min1) {
			depth = max1 - min2;
		}
		else {
			depth = max2 - min1;
		}

		if (depth < minDepth) {
			minDepth = depth;
			minAxis = axis;
		}
	}
	return false;
}

void Collisions::polygonPolygonResolution(
	ConvexPolygon& polygonA,
	ConvexPolygon& polygonB,
	float depth,
	sf::Vector2f axis
)
{
	sf::Vector2f overlapCorrection = 0.5f * axis * depth;

	polygonA.currentPosition -= overlapCorrection;
	polygonA.oldPosition -= overlapCorrection;

	polygonB.currentPosition += overlapCorrection;
	polygonB.oldPosition += overlapCorrection;

	std::vector<sf::Vector2f> contactPoints(2);
	int contactCount = 0;

	std::vector<sf::Vector2f> polygonAVertices = polygonA.getAllVertices();
	std::vector<sf::Vector2f> polygonBVertices = polygonB.getAllVertices();

	Collisions::findContactPoints(polygonAVertices, polygonBVertices, contactPoints[0], contactPoints[1], contactCount);

	std::vector<sf::Vector2f> r_PA_List(2);
	std::vector<sf::Vector2f> r_PB_List(2);
	std::vector<sf::Vector2f> impulseList(2);

	for (int i = 0; i < contactCount; i++)
	{
		sf::Vector2f r_PA = contactPoints[i] - polygonA.currentPosition;
		sf::Vector2f r_PB = contactPoints[i] - polygonB.currentPosition;

		r_PA_List[i] = r_PA;
		r_PB_List[i] = r_PB;

		sf::Vector2f v_A = (polygonA.currentPosition - polygonA.oldPosition);
		v_A -= (polygonA.currentAngle - polygonA.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(r_PA);

		sf::Vector2f v_B = (polygonB.currentPosition - polygonB.oldPosition);
		v_B -= (polygonB.currentAngle - polygonB.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(r_PB);

		float contactVelocity_mag = VectorMath::dotProduct(v_B - v_A, axis);

		if (contactVelocity_mag > 0.f)
		{
			continue;
		}

		float r_PA_PerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(r_PA), axis);
		float r_PB_PerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(r_PB), axis);

		float denom = (1.f / polygonA.mass) + (1.f / polygonB.mass) +
			(r_PA_PerpDotN * r_PA_PerpDotN) * (1.f / polygonA.momentOfInertia) +
			(r_PB_PerpDotN * r_PB_PerpDotN) * (1.f / polygonB.momentOfInertia);

		float restitutionCoefficient = (polygonA.resCoeff > polygonB.resCoeff ? polygonB.resCoeff : polygonA.resCoeff);

		float j = -(1.f + restitutionCoefficient) * contactVelocity_mag;
		j /= denom;
		j /= contactCount;

		sf::Vector2f impulse = j * axis;
		impulseList[i] = impulse;
	}

	for (int i = 0; i < contactCount; i++)
	{
		sf::Vector2f impulse = impulseList[i];
		sf::Vector2f r_PA = r_PA_List[i];
		sf::Vector2f r_PB = r_PB_List[i];

		polygonA.oldPosition += impulse * (1.f / polygonA.mass);
		polygonA.oldAngle += VectorMath::crossProduct(r_PA, impulse) * (1.f / polygonA.momentOfInertia) * VectorMath::radToDegFactor;

		polygonB.oldPosition -= impulse * (1.f / polygonB.mass);
		polygonB.oldAngle -= VectorMath::crossProduct(r_PB, impulse) * (1.f / polygonB.momentOfInertia) * VectorMath::radToDegFactor;
	}

	return;
}

sf::Vector2f Collisions::findContactPoint(
	Circle& c,
	ConvexPolygon& convexPolygon
)
{
	sf::Vector2f contactPoint;
	float distanceToContactPoint = VectorMath::upperLimit;

	int polygonVertexCount = convexPolygon.body.getPointCount();

	for (int i = 0; i < polygonVertexCount; ++i) {
		sf::Vector2f tempVector = VectorMath::closestPointOnLineSegment(c.currentPosition, convexPolygon.getVertexPosition(i),
			convexPolygon.getVertexPosition((i + 1) % polygonVertexCount));
		float tempDistance = VectorMath::findDistance(c.currentPosition, tempVector);

		if (tempDistance < distanceToContactPoint) {
			distanceToContactPoint = tempDistance;
			contactPoint = tempVector;
		}
	}
	return contactPoint;
}

void Collisions::findContactPoints(
	std::vector<sf::Vector2f>& shapeAVertices,
	std::vector<sf::Vector2f>& shapeBVertices,
	sf::Vector2f& contactPoint1,
	sf::Vector2f& contactPoint2,
	int& contactCount
)
{
	contactPoint1 = { -1.f, -1.f };
	contactPoint2 = { -1.f, -1.f };

	float minDist = VectorMath::upperLimit;
	int shapeAVertexCount = shapeAVertices.size();
	int shapeBVertexCount = shapeBVertices.size();

	for (int i = 0; i < shapeAVertexCount; i++)
	{
		sf::Vector2f point = shapeAVertices[i];

		for (int j = 0; j < shapeBVertexCount; j++)
		{
			sf::Vector2f v1 = shapeBVertices[j];
			sf::Vector2f v2 = shapeBVertices[(j + 1) % shapeBVertexCount];

			sf::Vector2f contactPoint = VectorMath::closestPointOnLineSegment(point, v1, v2);
			float dist = VectorMath::findDistance(contactPoint, point);

			if (VectorMath::nearlyEqual(dist, minDist))
			{
				if (!VectorMath::nearlyEqual(contactPoint, contactPoint1)) {
					contactPoint2 = contactPoint;
					contactCount = 2;
				}
			}

			else if (dist < minDist)
			{
				minDist = dist;
				contactCount = 1;
				contactPoint1 = contactPoint;
			}
		}
	}

	for (int i = 0; i < shapeBVertexCount; i++)
	{
		sf::Vector2f point = shapeBVertices[i];

		for (int j = 0; j < shapeAVertexCount; j++)
		{
			sf::Vector2f v1 = shapeAVertices[j];
			sf::Vector2f v2 = shapeAVertices[(j + 1) % shapeAVertexCount];

			sf::Vector2f contactPoint = VectorMath::closestPointOnLineSegment(point, v1, v2);
			float dist = VectorMath::findDistance(contactPoint, point);

			if (VectorMath::nearlyEqual(dist, minDist))
			{
				if (!VectorMath::nearlyEqual(contactPoint, contactPoint1)) {
					contactPoint2 = contactPoint;
					contactCount = 2;
				}
			}

			else if (dist < minDist)
			{
				minDist = dist;
				contactCount = 1;
				contactPoint1 = contactPoint;
			}
		}
	}
	return;
}

void Collisions::circleBarrierDetection(
	Circle& c,
	rectBarrier& b
)
{
	if (!Collisions::checkBoundingBox(c, b)) {
		return;
	}

	sf::Vector2f closestPoint;
	int barrierVertexCount = b.body.getPointCount();

	for (int i = 0; i < barrierVertexCount; ++i) {
		sf::Vector2f tempVector = VectorMath::closestPointOnLineSegment(c.currentPosition, b.vertexPositions[i],
			b.vertexPositions[(i + 1) % barrierVertexCount]);
		float tempDistance = VectorMath::findDistance(c.currentPosition, tempVector);

		if (tempDistance < c.size) {
			sf::Vector2f axis = tempVector - c.currentPosition;
			axis = VectorMath::normalise(axis);

			//Check the normal is facing in the correct direction
			if (VectorMath::dotProduct(b.position - c.currentPosition, axis) < 0.f) {
				axis = { -axis.x, -axis.y };
			}
			Collisions::circleBarrierResolution(c, b, (c.size - tempDistance), axis, tempVector - b.position);
		}
	}
}

void Collisions::circleBarrierResolution(
	Circle& c,
	rectBarrier& b,
	const float& depth,
	const sf::Vector2f& axis,
	const sf::Vector2f& contactPointOnBarrier
)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.
	//Denoting circle as A and barrier as B.
	c.currentPosition -= (depth * axis);
	c.oldPosition -= (depth * axis);

	sf::Vector2f circleToContactPoint = (contactPointOnBarrier + b.position) - c.currentPosition;

	sf::Vector2f circleVelocity = (c.currentPosition - c.oldPosition);
	circleVelocity -= (c.currentAngle - c.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(circleToContactPoint);

	float contactVelocityMag = VectorMath::dotProduct(-circleVelocity, axis);

	if (contactVelocityMag > 0.f) {
		return;
	}

	float raPerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(circleToContactPoint), axis);

	float denom = (1.f / c.mass) + 0.f +
		(raPerpDotN * raPerpDotN) * (1.f / c.momentOfInertia) +
		0.f;


	float j = -(1 + c.resCoeff) * contactVelocityMag;
	j /= denom;

	sf::Vector2f impulse = j * axis;

	c.oldPosition += impulse * (1.f / c.mass);
	c.oldAngle += VectorMath::crossProduct(circleToContactPoint, impulse) * (1.f / c.momentOfInertia) * VectorMath::radToDegFactor;

	return;
}

void Collisions::polygonBarrierDetection(
	ConvexPolygon& polygon,
	rectBarrier& barrier
)
{
	if (!Collisions::checkBoundingBox(polygon, barrier)) {
		return;
	}

	float minimumDepth = VectorMath::upperLimit;
	sf::Vector2f minimumAxis;

	sf::Vector2f barrierPosition = barrier.position;

	std::vector<sf::Vector2f> polygonVertices = polygon.getAllVertices();

	if ((projectionSAT(polygonVertices, barrier.vertexPositions, minimumDepth, minimumAxis)) ||
		(projectionSAT(barrier.vertexPositions, polygonVertices, minimumDepth, minimumAxis))) {
		return;
	}

	//Polygons are colliding and the minimum distance in order to separate the two has been found.

	//Check the normal is facing in the correct direction
	if (VectorMath::dotProduct(barrierPosition - polygon.currentPosition, minimumAxis) < 0.f) {
		minimumAxis = -minimumAxis;
	}
	polygonBarrierResolution(polygon, barrier, minimumDepth, minimumAxis);
}

void Collisions::polygonBarrierResolution(
	ConvexPolygon& polygon,
	rectBarrier& barrier,
	const float& depth,
	const sf::Vector2f& axis
)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.

	polygon.currentPosition -= (depth * axis);
	polygon.oldPosition -= (depth * axis);

	std::vector<sf::Vector2f> contactPoints(2);
	int contactCount = 0;

	std::vector<sf::Vector2f> polygonVertices = polygon.getAllVertices();

	Collisions::findContactPoints(polygonVertices, barrier.vertexPositions, contactPoints[0], contactPoints[1], contactCount);

	std::vector<sf::Vector2f> raList(2);
	std::vector<sf::Vector2f> impulseList(2);

	for (int i = 0; i < contactCount; i++)
	{
		sf::Vector2f polygonToContactPoint = contactPoints[i] - polygon.currentPosition;

		raList[i] = polygonToContactPoint;

		sf::Vector2f polygonVelocity = (polygon.currentPosition - polygon.oldPosition);

		polygonVelocity -= (polygon.currentAngle - polygon.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(polygonToContactPoint);

		float contactVelocityMag = VectorMath::dotProduct(-polygonVelocity, axis);

		if (contactVelocityMag > 0.f)
		{
			continue;
		}

		float raPerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(polygonToContactPoint), axis);

		float denom = (1.f / polygon.mass) + 0.f +
			(raPerpDotN * raPerpDotN) * (1.f / polygon.momentOfInertia) +
			0.f;

		float j = -(1.f + polygon.resCoeff) * contactVelocityMag;
		j /= denom;
		j /= contactCount;

		sf::Vector2f impulse = j * axis;
		impulseList[i] = impulse;
	}

	for (int i = 0; i < contactCount; i++)
	{
		sf::Vector2f impulse = impulseList[i];
		sf::Vector2f ra = raList[i];

		polygon.oldPosition += impulse * (1.f / polygon.mass);
		polygon.oldAngle += VectorMath::crossProduct(ra, impulse) * (1.f / polygon.momentOfInertia) * VectorMath::radToDegFactor;
	}

	return;
}