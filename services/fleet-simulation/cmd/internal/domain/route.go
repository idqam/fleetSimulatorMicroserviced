package domain

import (
	"time"

	"github.com/google/uuid"
)

type RouteGraph struct {
	ID            uuid.UUID
	Name          string
	Nodes         []Node
	AdjacencyList map[int][]Edge
	Seed          int64
	N             int
	P             float64
	CreatedAt     time.Time
	
}

type Route struct {
	ID            uuid.UUID
	Name          string
	GraphID       uuid.UUID
	NodeSequence  []int
	TotalDistance int64
	EstimatedTime float64
	CreatedAt     time.Time
}

type Node struct {
	ID        uuid.UUID
	X         int64
	Y         int64
	CreatedAt time.Time
}

type Edge struct {
	ID         uuid.UUID
	FromNodeID uuid.UUID
	ToNodeID   uuid.UUID
	Distance   int64
	TravelTime float64
	CreatedAt  time.Time
}