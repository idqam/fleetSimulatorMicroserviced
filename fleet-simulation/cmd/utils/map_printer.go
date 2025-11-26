package utils

import (
	"fmt"

	"fleetsimulator/m/cmd/internal/domain"

	"github.com/google/uuid"
)

// MapPrinter visualizes a RouteMap in the terminal using ASCII art.
type MapPrinter struct {
	rm        *domain.RouteMap
	cellSize  int
	nodeChar  string
	edgeChar  string
	emptyChar string
}

// NewMapPrinter creates a new terminal map printer.
func NewMapPrinter(rm *domain.RouteMap) *MapPrinter {
	return &MapPrinter{
		rm:        rm,
		cellSize:  3,
		nodeChar:  "●",
		edgeChar:  "─",
		emptyChar: " ",
	}
}

// Print outputs the map to stdout.
func (mp *MapPrinter) Print() {
	if len(mp.rm.Nodes) == 0 {
		fmt.Println("No nodes in map")
		return
	}

	// Find bounds
	minX, maxX, minY, maxY := mp.getBounds()
	width := int(maxX - minX + 1)
	height := int(maxY - minY + 1)

	// Create grid (scaled for terminal display)
	grid := mp.createGrid(width, height, minX, minY)

	// Place nodes
	for i, node := range mp.rm.Nodes {
		x := int(node.X - minX)
		y := int(node.Y - minY)
		if x >= 0 && x < width && y >= 0 && y < height {
			grid[y][x] = fmt.Sprintf("[%d]", i) // node index
		}
	}

	// Print header
	fmt.Printf("\n=== RouteMap: %s ===\n", mp.rm.Name)
	fmt.Printf("Nodes: %d | Edges: %d | Seed: %d | P: %.2f\n", mp.rm.N, mp.countEdges(), mp.rm.Seed, mp.rm.P)
	fmt.Println()

	// Print grid
	for y := height - 1; y >= 0; y-- { // reverse Y for top-down display
		fmt.Printf("%3d | ", minY+int64(y))
		for x := 0; x < width; x++ {
			fmt.Printf("%-4s", grid[y][x])
		}
		fmt.Println()
	}

	// Print X axis
	fmt.Print("    +")
	for x := 0; x < width; x++ {
		fmt.Printf("----")
	}
	fmt.Println()
	fmt.Print("    | ")
	for x := 0; x < width; x++ {
		fmt.Printf("%-4d", minX+int64(x))
	}
	fmt.Println()

	// Print node details and adjacency info
	fmt.Println("\n=== Node Details ===")
	for i, node := range mp.rm.Nodes {
		fmt.Printf("Node[%d]: ID=%s, Pos=(%d,%d)\n", i, node.ID.String()[:8], node.X, node.Y)

		// Print edges from this node
		if edges, ok := mp.rm.AdjacencyList[i]; ok && len(edges) > 0 {
			fmt.Printf("  Edges (%d):\n", len(edges))
			for _, edge := range edges {
				// Find target node index
				targetIdx := mp.findNodeIndex(edge.ToNodeID)
				fmt.Printf("    → Node[%d] (ID=%s): distance=%d KM, time=%.2f h\n",
					targetIdx, edge.ToNodeID.String()[:8], edge.Distance, edge.TravelTime)
			}
		}
	}
}

// getBounds finds the min/max coordinates in the map.
func (mp *MapPrinter) getBounds() (minX, maxX, minY, maxY int64) {
	if len(mp.rm.Nodes) == 0 {
		return 0, 0, 0, 0
	}
	minX, maxX = mp.rm.Nodes[0].X, mp.rm.Nodes[0].X
	minY, maxY = mp.rm.Nodes[0].Y, mp.rm.Nodes[0].Y

	for _, node := range mp.rm.Nodes {
		if node.X < minX {
			minX = node.X
		}
		if node.X > maxX {
			maxX = node.X
		}
		if node.Y < minY {
			minY = node.Y
		}
		if node.Y > maxY {
			maxY = node.Y
		}
	}
	return
}

// createGrid initializes a display grid.
func (mp *MapPrinter) createGrid(width, height int, minX, minY int64) [][]string {
	grid := make([][]string, height)
	for y := 0; y < height; y++ {
		grid[y] = make([]string, width)
		for x := 0; x < width; x++ {
			grid[y][x] = mp.emptyChar
		}
	}
	return grid
}

// findNodeIndex returns the index of a node by UUID.
func (mp *MapPrinter) findNodeIndex(id uuid.UUID) int {
	for i, node := range mp.rm.Nodes {
		if node.ID == id {
			return i
		}
	}
	return -1
}

// countEdges counts total edges in the adjacency list.
func (mp *MapPrinter) countEdges() int {
	count := 0
	for _, edges := range mp.rm.AdjacencyList {
		count += len(edges)
	}
	return count
}

// PrintSummary prints a brief summary of the RouteMap.
func (mp *MapPrinter) PrintSummary() {
	fmt.Printf("\n=== RouteMap Summary ===\n")
	fmt.Printf("Name:  %s\n", mp.rm.Name)
	fmt.Printf("Nodes: %d\n", len(mp.rm.Nodes))
	fmt.Printf("Edges: %d (directed)\n", mp.countEdges())
	fmt.Printf("ER Params: n=%d, p=%.4f, seed=%d\n", mp.rm.N, mp.rm.P, mp.rm.Seed)

	if len(mp.rm.Nodes) > 0 {
		minX, maxX, minY, maxY := mp.getBounds()
		fmt.Printf("Bounds: X=[%d, %d], Y=[%d, %d]\n", minX, maxX, minY, maxY)
	}
	fmt.Println()
}
