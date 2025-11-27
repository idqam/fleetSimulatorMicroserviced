package service

import (
	"fleetsimulator/m/cmd/internal/domain"
	"fmt"
	"math/rand"
	"strconv"
	"time"

	"github.com/google/uuid"
)

func pickValidSpawnNode(g domain.RouteGraph) (int, domain.Node) {
	for {
		i := rand.Intn(len(g.Nodes))
		if len(g.AdjacencyList[i]) > 0 {
			return i, g.Nodes[i]
		}
	}
}

func GenerateVehicleRandomLocation(g domain.RouteGraph) domain.Vehicle {

	id, err := uuid.NewUUID()
	if err != nil {
		panic("Error creating Vehicle UUID")
	}

	speed := rand.Float64()*10 + 5

	_, node := pickValidSpawnNode(g)

	return domain.Vehicle{
		ID:               id,
		Type:             "Car",
		Speed:            speed,
		StartingLocation: node.ID,
		CurrentLocation:  node.ID,
		PosX:             float64(node.X),
		PosY:             float64(node.Y),
		Status:           domain.StatusIdle,
		CreatedAt:        time.Now(),
	}
}


func GenerateFleet(g domain.RouteGraph, fleetSize int) domain.Fleet {

	id, err := uuid.NewUUID()
	if err != nil {
		panic("Error creating Fleet UUID")
	}

	name := "Fleet " + strconv.Itoa(rand.Intn(100))

	vehicleList := make([]domain.Vehicle, 0, fleetSize)

	for i := 0; i < fleetSize; i++ {
		vehicleList = append(vehicleList, GenerateVehicleRandomLocation(g))
	}

	return domain.Fleet{
		ID:       id,
		Name:     name,
		Vehicles: vehicleList,
	}
}

func PrintFleetList(f *domain.Fleet) {
	fmt.Printf("Fleet: %s (%s)\n", f.Name, f.ID.String())
	for _, v := range f.Vehicles {
		fmt.Printf("  Vehicle %s at (%.2f, %.2f)\n",
			v.ID.String(), v.PosX, v.PosY)
	}
}
