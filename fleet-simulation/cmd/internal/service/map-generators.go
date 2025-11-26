package service

import (
	"math/rand"
	"time"

	"fleetsimulator/m/cmd/internal/domain"

	"github.com/google/uuid"
)

func generateNodeID() uuid.UUID {
	return uuid.New()
}

func currentTimestamp() time.Time {
	return time.Now()
}

func GenerateNodes(n int ) []domain.Node {
	nodes := make([]domain.Node, n)
	for i := 0; i < n; i++ {
		nodes[i] = domain.Node{
			ID:        generateNodeID(),
			X:         int64(i * 10),
			Y:         int64(i * 5),
			CreatedAt: currentTimestamp(),
		}
	}
	return nodes
}

func ErdosReniyiGraph(nodes []domain.Node, p float64, seed int64) map[int][]domain.Edge {
	adjacencyList := make(map[int][]domain.Edge)
	numNodes := len(nodes)
	
	r := rand.New(rand.NewSource(seed))

	for i := 0; i < numNodes; i++ {
		for j := i + 1; j < numNodes; j++ { // undirected: only consider j>i and add both directions
			if r.Float64() < p {
				distance := calculateDistance(nodes[i], nodes[j])
				travelTime := float64(distance) / 60.0

				edgeIJ := domain.Edge{
					ID:         uuid.New(),
					FromNodeID: nodes[i].ID,
					ToNodeID:   nodes[j].ID,
					Distance:   distance,
					TravelTime: travelTime,
					CreatedAt:  currentTimestamp(),
				}
				edgeJI := domain.Edge{
					ID:         uuid.New(),
					FromNodeID: nodes[j].ID,
					ToNodeID:   nodes[i].ID,
					Distance:   distance,
					TravelTime: travelTime,
					CreatedAt:  currentTimestamp(),
				}

				adjacencyList[i] = append(adjacencyList[i], edgeIJ)
				adjacencyList[j] = append(adjacencyList[j], edgeJI)
			}
		}
	}
	return adjacencyList
}

func abs(x int64) int64 {
	if x < 0 {
		return -x
	}
	return x
}

func calculateDistance(node1, node2 domain.Node) int64 {
	return abs(node2.X - node1.X) + abs(node2.Y - node1.Y)
}

func GenerateRouteMap(name string, nodes []domain.Node, p float64, seed int64) domain.RouteMap {
	adjacency := ErdosReniyiGraph(nodes, p, seed)
	rm := domain.RouteMap{
		ID:            uuid.New(),
		Name:          name,
		Nodes:         nodes,
		AdjacencyList: adjacency,
		Routes:        []domain.Route{},
		CreatedAt:     currentTimestamp(),
		Seed:          seed,
		N:             len(nodes),
		P:             p,
	}
	return rm
}