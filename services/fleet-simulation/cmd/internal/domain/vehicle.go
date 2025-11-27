package domain

import (
	"time"

	"github.com/google/uuid"
)

type Vehicle struct {
	ID                uuid.UUID
	Type              string
	Speed             float64
	StartingLocation  uuid.UUID
	TargetLocation    uuid.UUID
	CurrentLocation   uuid.UUID
	CurrentRouteID    *uuid.UUID
	NextWaypointID    *uuid.UUID
	CurrentEdgeID     *uuid.UUID
	EdgeProgress      float64
	CreatedAt         time.Time
	Status            string
}

type Fleet struct {
	ID       uuid.UUID
	Name     string
	Vehicles []Vehicle
}

type VehiclePosition struct {
	X float64
	Y float64
}

const (
	StatusIdle      = "idle"
	StatusActive    = "active"
	StatusCompleted = "completed"
	StatusCharging  = "charging"
)