package domain

import (
	"time"

	"github.com/google/uuid"
)

type Vehicle struct {
    ID uuid.UUID  `json:"id"`
    Type string `json:"type"`
    Speed float64 `json:"speed"`
    StartingLocation uuid.UUID `json:"starting_location"`
    TargetLocation uuid.UUID `json:"target_location"`
    CurrentLocation uuid.UUID `json:"current_location"`
    CreatedAt time.Time `json:"created_at"`
    Status string `json:"status"`
}

type Fleet struct {
    ID uuid.UUID  `json:"id"`
    Name string `json:"name"`
    Vehicles []Vehicle `json:"vehicles"`
}

type EventMetadata struct {
    Timestamp int64 `json:"timestamp"`
    Version   int   `json:"version"`
}

type VehicleEvent struct {
    EventType    string         `json:"event_type"`     
    VehicleID    uuid.UUID      `json:"vehicle_id"`
    Timestamp    int64          `json:"timestamp"`
    Version      int            `json:"version"`
    Payload      VehicleUpdate  `json:"payload"`
}

type VehicleUpdate struct {
    ID               uuid.UUID     `json:"id"`                 
    CurrentLocation  *uuid.UUID    `json:"current_location,omitempty"`
    Status           *string       `json:"status,omitempty"`
    Speed            *float64      `json:"speed,omitempty"`
    TargetLocation   *uuid.UUID    `json:"target_location,omitempty"`
    Timestamp        int64         `json:"timestamp"`           
    Version          int           `json:"version"`             
}

const (
    StatusActive   = "active"
    StatusInactive = "inactive"
    StatusCompleted = "completed"
)