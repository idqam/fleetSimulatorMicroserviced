package domain

import (
	"time"

	"github.com/google/uuid"
)

// Route represents a predefined path that vehicles can follow.
// Graph Based implementation using discrete waypoints connected by edges.
// A node will be (X, Y) coordinates representing a waypoint.
// A node will have a type for "fuel", "rest stop", "delivery point", etc.
// but the type will be added later to keep complexity down for now.
//An edge will represent the distance and travel time between two nodes.
// Future enhancements can include traffic data, road conditions, etc.
// a node can have multiple edges to other nodes.
// The graph will be stored as an adjacency list for efficient traversal.
// A route will be a sequence of nodes representing the path to follow.
// A map of routes can be maintained for different scenarios which is a list of routes.
//So the map is a collection of adjacency lists representing different routes.

type RouteMap struct {
	ID            uuid.UUID            `json:"id"`
	Name          string               `json:"name"`
	Nodes         []Node               `json:"nodes"`
	AdjacencyList map[string][]Edge    `json:"adjacency_list"` // nodeID -> []edges (keys are node ID strings)
	Routes        []Route              `json:"routes"`
	CreatedAt     time.Time            `json:"created_at"`
}

type Route struct {
	ID            uuid.UUID    `json:"id"`
	Name          string       `json:"name"`
	RouteMapID    uuid.UUID    `json:"route_map_id"`    // FK to RouteMap
	WaypointIDs   []uuid.UUID  `json:"waypoint_ids"`    // references Node IDs in RouteMap
	TotalDistance float64      `json:"total_distance"` //KM
	EstimatedTime float64      `json:"estimated_time"` // in hours will be sum of edge travel times with some random variation
	CreatedAt     time.Time    `json:"created_at"`
}

type Node struct {
	ID        uuid.UUID  `json:"id"`
	X         float64    `json:"x"` // discrete coordinate system
	Y         float64    `json:"y"`
	CreatedAt time.Time  `json:"created_at"`
}
type Edge struct {
	ID         uuid.UUID  `json:"id"`
	FromNodeID uuid.UUID  `json:"from_node_id"`
	ToNodeID   uuid.UUID  `json:"to_node_id"`
	Distance   float64    `json:"distance"`    // in KM
	TravelTime float64    `json:"travel_time"` // in hours
	CreatedAt  time.Time  `json:"created_at"`
}