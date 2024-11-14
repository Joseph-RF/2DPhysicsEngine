#include "VectorMath.h"

const float VectorMath::upperLimit = std::numeric_limits<float>::max();
const float VectorMath::lowerLimit = std::numeric_limits<float>::lowest();
const float VectorMath::pi = 3.141592;
const float VectorMath::root3 = 1.732051;

float VectorMath::dotProduct(const sf::Vector2f& v1, const sf::Vector2f& v2)
{
	return ((v1.x * v2.x) + (v1.y * v2.y));
}

float VectorMath::crossProduct(const sf::Vector2f& v1, const sf::Vector2f& v2)
{
	return (v1.x * v2.y - v1.y * v2.x);
}

sf::Vector2f VectorMath::normalise(const sf::Vector2f& v)
{
	return sf::Vector2f(v / pow(v.x * v.x + v.y * v.y, 0.5f));
}

float VectorMath::findDistance(const sf::Vector2f& v1, const sf::Vector2f& v2)
{
	return pow(((v2.x - v1.x) * (v2.x - v1.x)) + ((v2.y - v1.y) * (v2.y - v1.y)), 0.5f);
}

sf::Vector2f VectorMath::perpendicular(const sf::Vector2f& v)
{
	return { v.y, -v.x };
}

bool VectorMath::nearlyEqual(const float a1, const float a2)
{
	float verySmallAmount = 0.1f;
	return abs(a1 - a2) < verySmallAmount;
}

bool VectorMath::nearlyEqual(const sf::Vector2f& v1, const sf::Vector2f& v2)
{
	float verySmallAmount = 0.1f;
	return VectorMath::findDistance(v1, v2) < verySmallAmount;
}

sf::Vector2f VectorMath::closestPointOnLineSegment(
	const sf::Vector2f& referencePoint,
	const sf::Vector2f& vertex1Pos,
	const sf::Vector2f& vertex2Pos
)
{
	sf::Vector2f edge = vertex2Pos - vertex1Pos;
	float denominator = VectorMath::dotProduct(edge, edge);
	if (denominator == 0.0f) {
		return vertex1Pos;
	}

	float t = VectorMath::dotProduct((referencePoint - vertex1Pos), edge) / denominator;

	//Clamp t between 0 and 1.
	if (t > 1.f) { t = 1.f; }
	else if (t < 0.f) { t = 0.f; }

	return (vertex1Pos + (t * (vertex2Pos - vertex1Pos)));
}