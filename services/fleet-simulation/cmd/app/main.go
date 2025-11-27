package main

import (
	"fleetsimulator/m/cmd/internal/domain"
	"fleetsimulator/m/cmd/internal/service"
	"fmt"
)

func main() {


	graph := service.PickMapByAlgorithm(3, 1000, 1000, 12345)

	fmt.Println("=== Generated Map ===")
	fmt.Println("Name:", graph.Name)
	fmt.Println("Nodes:", len(graph.Nodes))
	fmt.Println("Edges:", countEdges(graph))

	
	fleet := service.GenerateFleet(graph, 5)

	fmt.Println("\n=== Fleet Information ===")
	service.PrintFleetList(&fleet)
}


func countEdges(g domain.RouteGraph) int {
	count := 0
	for _, edges := range g.AdjacencyList {
		count += len(edges)
	}
	return count
}
