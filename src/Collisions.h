#pragma once

#include <SFML/System.hpp>

#include "Shapes.h"
#include "Engine.h"

#include <vector>

class Circle;
class ConvexPolygon;
class rectBarrier;

namespace Collisions {

	bool checkBoundingBox(
		Circle& circle,
		rectBarrier& barrier
	);
	bool checkBoundingBox(
		ConvexPolygon& polygon,
		rectBarrier& barrier
	);
	bool checkBoundingBox(
		Circle& circle,
		ConvexPolygon& polygon
	);
	bool checkBoundingBox(
		ConvexPolygon& polygonA,
		ConvexPolygon& polygonB
	);

	void circleCircleDetection(
		Circle& c1,
		Circle& c2
	);
	void circleCircleResolution(
		Circle& c1,
		Circle& c2,
		float depth,
		sf::Vector2f axis
	);

	void circlePolygonDetection(
		Circle& c,
		ConvexPolygon& polygon
	);
	void circlePolygonResolution(
		Circle& c,
		ConvexPolygon& polygon,
		float depth,
		sf::Vector2f axis,
		const sf::Vector2f& contactPointOnPolygon
	);

	void polygonPolygonDetection(
		ConvexPolygon& polygonA,
		ConvexPolygon& polygonB
	);
	bool projectionSAT(
		std::vector<sf::Vector2f>& polygonAVertices,
		std::vector<sf::Vector2f>& polygonBVertices,
		float& minDepth,
		sf::Vector2f& minAxis
	);
	void polygonPolygonResolution(
		ConvexPolygon& polygonA,
		ConvexPolygon& polygonB,
		float depth,
		sf::Vector2f axis
	);

	sf::Vector2f findContactPoint(
		Circle& c,
		ConvexPolygon& convexPolygon
	);

	void findContactPoints(
		std::vector<sf::Vector2f>& shapeAVertices,
		std::vector<sf::Vector2f>& shapeBVertices,
		sf::Vector2f& contactPoint1,
		sf::Vector2f& contactPoint2,
		int& contactCount
	);

	void circleBarrierDetection(
		Circle& c,
		rectBarrier& b
	);
	void circleBarrierResolution(
		Circle& c,
		rectBarrier& b,
		const float& depth,
		const sf::Vector2f& axis,
		const sf::Vector2f& contactPointOnBarrier
	);

	void polygonBarrierDetection(
		ConvexPolygon& polygon,
		rectBarrier& barrier
	);
	void polygonBarrierResolution(
		ConvexPolygon& polygon,
		rectBarrier& barrier,
		const float& depth,
		const sf::Vector2f& axis
	);
}