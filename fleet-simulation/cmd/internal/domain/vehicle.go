package domain

type Vehicle struct {
	ID string `json:"id"`
	Type string `json:"type"`
	Speed float64 `json:"speed"`
	StartingLocation string `json:"starting_location"`
	TargetLocation string `json:"target_location"`
	CurrentLocation string `json:"current_location"`
	CreatedAt int64 `json:"created_at"`
	Status string `json:"status"`
}

type Fleet struct {
	ID string `json:"id"`
	Name string `json:"name"`
	Vehicles []Vehicle `json:"vehicles"`
}

type EventMetadata struct {
    Timestamp int64 `json:"timestamp"`
    Version   int   `json:"version"`
}

type VehicleEvent struct {
    EventType    string         `json:"event_type"`     
    VehicleID    string         `json:"vehicle_id"`
    Timestamp    int64          `json:"timestamp"`
    Version      int            `json:"version"`
    Payload      VehicleUpdate  `json:"payload"`
}
type VehicleUpdate struct {
    ID               string    `json:"id"`                 
    CurrentLocation  *string   `json:"current_location,omitempty"`
    Status           *string   `json:"status,omitempty"`
    Speed            *float64  `json:"speed,omitempty"`
    TargetLocation   *string   `json:"target_location,omitempty"`
    Timestamp        int64     `json:"timestamp"`           
    Version          int       `json:"version"`             
}

const (
    StatusActive   = "active"
    StatusInactive = "inactive"
    StatusCompleted = "completed"
)