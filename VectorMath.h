#pragma once

#include <SFML/System.hpp>

#include <limits>

namespace VectorMath {

	extern const float upperLimit;
	extern const float lowerLimit;
	extern const float pi;
	extern const float root3;

	float dotProduct(const sf::Vector2f& v1, const sf::Vector2f& v2);
	float crossProduct(const sf::Vector2f& v1, const sf::Vector2f& v2);
	sf::Vector2f normalise(const sf::Vector2f& v);
	float findDistance(const sf::Vector2f& v1, const sf::Vector2f& v2);
	sf::Vector2f perpendicular(const sf::Vector2f& v);
	bool nearlyEqual(const float a1, const float a2);
	bool nearlyEqual(const sf::Vector2f& v1, const sf::Vector2f& v2);

	sf::Vector2f closestPointOnLineSegment(
		const sf::Vector2f& referencePoint,
		const sf::Vector2f& vertex1Pos,
		const sf::Vector2f& vertex2Pos
	);
}
