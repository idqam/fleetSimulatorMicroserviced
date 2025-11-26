package main

import (
	"fmt"

	"fleetsimulator/m/cmd/internal/service"
	"fleetsimulator/m/cmd/utils"
)

func main() {
	fmt.Println("Fleet Simulator - Map Visualization")
	fmt.Println("===================================")

	// Generate nodes using a simple grid
	n := 100
	nodes := service.GenerateNodes(n)
	fmt.Printf("Generated %d nodes\n", len(nodes))

	// Generate ER random graph with provenance
	seed := int64(42)
	p := 0.4 // probability of edge
	routeMap := service.GenerateRouteMap("Test Map", nodes, p, seed)

	// Print summary
	printer := utils.NewMapPrinter(&routeMap)
	printer.PrintSummary()

	// Print detailed map visualization
	printer.Print()
}
