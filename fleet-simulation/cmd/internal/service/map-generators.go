package service

import (
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
			X:         float64(i * 10),
			Y:         float64(i * 5),
			CreatedAt: currentTimestamp(),
		}
	}
	return nodes
}