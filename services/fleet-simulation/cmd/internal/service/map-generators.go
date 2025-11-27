package service

import (
	"math"
	"math/rand"
	"sort"
	"time"

	"fleetsimulator/m/cmd/internal/domain"

	"github.com/google/uuid"
)

type edge struct {
	from, to int
	dist     float64
}

// GenerateNodes creates n random nodes within bounds
func GenerateNodes(n int) []domain.Node {
	return GenerateNodesWithBounds(n, 500, 500)
}

// GenerateNodesWithBounds creates n random nodes within specified width and height
func GenerateNodesWithBounds(n int, width, height int64) []domain.Node {
	nodes := make([]domain.Node, n)
	r := rand.New(rand.NewSource(time.Now().UnixNano()))
	for i := 0; i < n; i++ {
		nodes[i] = domain.Node{
			ID:        uuid.New(),
			X:         int64(r.Intn(int(width))),
			Y:         int64(r.Intn(int(height))),
			CreatedAt: time.Now(),
		}
	}
	return nodes
}

// ============================================================================
// Algorithm 1: Poisson Disc Sampling + Delaunay Triangulation → Pruned Graph
// ============================================================================

func PoissonDiscSamplingMap(width, height, minDistance int64, seed int64, pruneThreshold float64) domain.RouteGraph {
	r := rand.New(rand.NewSource(seed))
	nodes := poissonDiscSampling(width, height, minDistance, r)
	cellSize := minDistance / 2
	triangles := delaunayTriangulation(nodes, cellSize)
	adjacency := triangulationToGraph(nodes, triangles, pruneThreshold)

	return domain.RouteGraph{
		ID:            uuid.New(),
		Name:          "Poisson Disc + Delaunay Pruned",
		Nodes:         nodes,
		AdjacencyList: adjacency,
		CreatedAt:     time.Now(),
		Seed:          seed,
		N:             len(nodes),
		P:             0,
	}
}

func poissonDiscSampling(width, height, minDist int64, r *rand.Rand) []domain.Node {
	cellSize := float64(minDist) / math.Sqrt(2)
	gridWidth := int(math.Ceil(float64(width) / cellSize))
	gridHeight := int(math.Ceil(float64(height) / cellSize))
	grid := make([][]int, gridWidth)
	for i := range grid {
		grid[i] = make([]int, gridHeight)
		for j := range grid[i] {
			grid[i][j] = -1
		}
	}

	nodes := []domain.Node{}
	active := []int{}

	firstNode := domain.Node{
		ID:        uuid.New(),
		X:         int64(r.Intn(int(width))),
		Y:         int64(r.Intn(int(height))),
		CreatedAt: time.Now(),
	}
	nodes = append(nodes, firstNode)
	active = append(active, 0)
	grid[int(firstNode.X/cellSize64(cellSize))][int(firstNode.Y/cellSize64(cellSize))] = 0

	for len(active) > 0 {
		idx := r.Intn(len(active))
		currentIdx := active[idx]
		current := nodes[currentIdx]
		found := false

		for attempt := 0; attempt < 30; attempt++ {
			angle := 2 * math.Pi * r.Float64()
			dist := float64(minDist) * (1 + r.Float64())
			newX := current.X + int64(dist*math.Cos(angle))
			newY := current.Y + int64(dist*math.Sin(angle))

			if newX >= 0 && newX < width && newY >= 0 && newY < height {
				gridX := int(float64(newX) / cellSize)
				gridY := int(float64(newY) / cellSize)

				if isValidSpot(grid, gridX, gridY, int(minDist), nodes, newX, newY) {
					newNode := domain.Node{
						ID:        uuid.New(),
						X:         newX,
						Y:         newY,
						CreatedAt: time.Now(),
					}
					nodes = append(nodes, newNode)
					active = append(active, len(nodes)-1)
					grid[gridX][gridY] = len(nodes) - 1
					found = true
					break
				}
			}
		}

		if !found {
			active = append(active[:idx], active[idx+1:]...)
		}
	}
	return nodes
}

func cellSize64(cs float64) int64 {
	return int64(cs)
}

func isValidSpot(grid [][]int, x, y, minDist int, nodes []domain.Node, newX, newY int64) bool {
	searchRadius := 2
	for i := -searchRadius; i <= searchRadius; i++ {
		for j := -searchRadius; j <= searchRadius; j++ {
			nx, ny := x+i, y+j
			if nx >= 0 && nx < len(grid) && ny >= 0 && ny < len(grid[0]) {
				if grid[nx][ny] != -1 {
					idx := grid[nx][ny]
					dist := euclideanDistance(nodes[idx].X, nodes[idx].Y, newX, newY)
					if dist < int64(minDist) {
						return false
					}
				}
			}
		}
	}
	return true
}

func delaunayTriangulation(nodes []domain.Node, cellSize int64) [][]int {
	if len(nodes) < 3 {
		return [][]int{}
	}
	// Simplified Delaunay using Bowyer-Watson
	triangles := [][]int{}
	superTriangleIdx := addSuperTriangle(nodes)
	triangles = append(triangles, []int{superTriangleIdx, superTriangleIdx + 1, superTriangleIdx + 2})

	for i := 0; i < len(nodes)-3; i++ {
		badTriangles := []int{}
		for ti, tri := range triangles {
			if isPointInCircumcircle(nodes[i], nodes[tri[0]], nodes[tri[1]], nodes[tri[2]]) {
				badTriangles = append(badTriangles, ti)
			}
		}

		for i := len(badTriangles) - 1; i >= 0; i-- {
			triangles = append(triangles[:badTriangles[i]], triangles[badTriangles[i]+1:]...)
		}
	}
	return triangles
}

func addSuperTriangle(nodes []domain.Node) int {
	return len(nodes)
}

func isPointInCircumcircle(p, a, b, c domain.Node) bool {
	ax, ay := float64(a.X), float64(a.Y)
	bx, by := float64(b.X), float64(b.Y)
	cx, cy := float64(c.X), float64(c.Y)
	px, py := float64(p.X), float64(p.Y)

	det := (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)
	if math.Abs(det) < 1e-10 {
		return false
	}

	ux := ((ax*ax+ay*ay)*(cy-by) + (bx*bx+by*by)*(ay-cy) + (cx*cx+cy*cy)*(by-ay)) / (2 * det)
	uy := ((ax*ax+ay*ay)*(bx-cx) + (bx*bx+by*by)*(cx-ax) + (cx*cx+cy*cy)*(ax-bx)) / (2 * det)

	r := math.Sqrt((ax-ux)*(ax-ux) + (ay-uy)*(ay-uy))
	dist := math.Sqrt((px-ux)*(px-ux) + (py-uy)*(py-uy))

	return dist < r
}

func triangulationToGraph(nodes []domain.Node, triangles [][]int, pruneThreshold float64) map[int][]domain.Edge {
	adjacency := make(map[int][]domain.Edge)
	edgeSet := make(map[[2]int]bool)

	for _, tri := range triangles {
		for i := 0; i < 3; i++ {
			from, to := tri[i], tri[(i+1)%3]
			if from > to {
				from, to = to, from
			}
			edgeSet[[2]int{from, to}] = true
		}
	}

	for edge := range edgeSet {
		distance := euclideanDistance(nodes[edge[0]].X, nodes[edge[0]].Y, nodes[edge[1]].X, nodes[edge[1]].Y)
		if float64(distance) > pruneThreshold {
			continue
		}

		travelTime := float64(distance) / 60.0
		edgeObj := domain.Edge{
			ID:         uuid.New(),
			FromNodeID: nodes[edge[0]].ID,
			ToNodeID:   nodes[edge[1]].ID,
			Distance:   distance,
			TravelTime: travelTime,
			CreatedAt:  time.Now(),
		}

		adjacency[edge[0]] = append(adjacency[edge[0]], edgeObj)
		edgeObj.FromNodeID, edgeObj.ToNodeID = edgeObj.ToNodeID, edgeObj.FromNodeID
		edgeObj.ID = uuid.New()
		adjacency[edge[1]] = append(adjacency[edge[1]], edgeObj)
	}

	return adjacency
}

// ============================================================================
// Algorithm 2: Geometric Random Graph (radius-based)
// ============================================================================

func GeometricRandomGraphMap(numNodes int, radius float64, seed int64) domain.RouteGraph {
	return GeometricRandomGraphMapWithBounds(numNodes, radius, 500, 500, seed)
}

func GeometricRandomGraphMapWithBounds(numNodes int, radius float64, width, height int64, seed int64) domain.RouteGraph {
	r := rand.New(rand.NewSource(seed))
	nodes := generateRandomNodesWithBounds(numNodes, width, height, r)
	adjacency := geometricRandomGraphConnections(nodes, radius)

	return domain.RouteGraph{
		ID:            uuid.New(),
		Name:          "Geometric Random Graph (Radius-based)",
		Nodes:         nodes,
		AdjacencyList: adjacency,
		CreatedAt:     time.Now(),
		Seed:          seed,
		N:             numNodes,
		P:             radius / float64(width*height),
	}
}

func generateRandomNodes(count int, maxX, maxY int64, r *rand.Rand) []domain.Node {
	nodes := make([]domain.Node, count)
	for i := 0; i < count; i++ {
		nodes[i] = domain.Node{
			ID:        uuid.New(),
			X:         int64(r.Intn(int(maxX))),
			Y:         int64(r.Intn(int(maxY))),
			CreatedAt: time.Now(),
		}
	}
	return nodes
}

func generateRandomNodesWithBounds(count int, width, height int64, r *rand.Rand) []domain.Node {
	return generateRandomNodes(count, width, height, r)
}

func geometricRandomGraphConnections(nodes []domain.Node, radius float64) map[int][]domain.Edge {
	adjacency := make(map[int][]domain.Edge)
	radiusSq := radius * radius

	for i := 0; i < len(nodes); i++ {
		for j := i + 1; j < len(nodes); j++ {
			dist := euclideanDistance(nodes[i].X, nodes[i].Y, nodes[j].X, nodes[j].Y)
			if float64(dist*dist) <= radiusSq {
				travelTime := float64(dist) / 60.0

				edge1 := domain.Edge{
					ID:         uuid.New(),
					FromNodeID: nodes[i].ID,
					ToNodeID:   nodes[j].ID,
					Distance:   dist,
					TravelTime: travelTime,
					CreatedAt:  time.Now(),
				}
				edge2 := domain.Edge{
					ID:         uuid.New(),
					FromNodeID: nodes[j].ID,
					ToNodeID:   nodes[i].ID,
					Distance:   dist,
					TravelTime: travelTime,
					CreatedAt:  time.Now(),
				}

				adjacency[i] = append(adjacency[i], edge1)
				adjacency[j] = append(adjacency[j], edge2)
			}
		}
	}
	return adjacency
}

// ============================================================================
// Algorithm 3: Erdős–Rényi with Spatial Constraints
// ============================================================================

func ErdosRenyiSpatialMap(numNodes int, p float64, maxDistance int64, seed int64) domain.RouteGraph {
	return ErdosRenyiSpatialMapWithBounds(numNodes, p, maxDistance, 500, 500, seed)
}

func ErdosRenyiSpatialMapWithBounds(numNodes int, p float64, maxDistance, width, height int64, seed int64) domain.RouteGraph {
	r := rand.New(rand.NewSource(seed))
	nodes := generateRandomNodesWithBounds(numNodes, width, height, r)
	adjacency := erdosRenyiWithConstraints(nodes, p, maxDistance, r)

	return domain.RouteGraph{
		ID:            uuid.New(),
		Name:          "Erdős–Rényi with Spatial Constraints",
		Nodes:         nodes,
		AdjacencyList: adjacency,
		CreatedAt:     time.Now(),
		Seed:          seed,
		N:             numNodes,
		P:             p,
	}
}

func erdosRenyiWithConstraints(nodes []domain.Node, p float64, maxDist int64, r *rand.Rand) map[int][]domain.Edge {
	adjacency := make(map[int][]domain.Edge)

	for i := 0; i < len(nodes); i++ {
		for j := i + 1; j < len(nodes); j++ {
			if r.Float64() < p {
				dist := euclideanDistance(nodes[i].X, nodes[i].Y, nodes[j].X, nodes[j].Y)
				if dist <= maxDist {
					travelTime := float64(dist) / 60.0

					edge1 := domain.Edge{
						ID:         uuid.New(),
						FromNodeID: nodes[i].ID,
						ToNodeID:   nodes[j].ID,
						Distance:   dist,
						TravelTime: travelTime,
						CreatedAt:  time.Now(),
					}
					edge2 := domain.Edge{
						ID:         uuid.New(),
						FromNodeID: nodes[j].ID,
						ToNodeID:   nodes[i].ID,
						Distance:   dist,
						TravelTime: travelTime,
						CreatedAt:  time.Now(),
					}

					adjacency[i] = append(adjacency[i], edge1)
					adjacency[j] = append(adjacency[j], edge2)
				}
			}
		}
	}
	return adjacency
}

// ============================================================================
// Algorithm 4: Road L-System (for Urban Patterns)
// ============================================================================

func RoadLSystemMap(iterations int, seed int64) domain.RouteGraph {
	return RoadLSystemMapWithBounds(iterations, 500, 500, seed)
}

func RoadLSystemMapWithBounds(iterations int, width, height int64, seed int64) domain.RouteGraph {
	r := rand.New(rand.NewSource(seed))
	nodes, adjacency := generateRoadLSystemWithBounds(iterations, width, height, r)

	return domain.RouteGraph{
		ID:            uuid.New(),
		Name:          "Road L-System (Urban Patterns)",
		Nodes:         nodes,
		AdjacencyList: adjacency,
		CreatedAt:     time.Now(),
		Seed:          seed,
		N:             len(nodes),
		P:             0,
	}
}

func generateRoadLSystemWithBounds(iterations int, width, height int64, r *rand.Rand) ([]domain.Node, map[int][]domain.Edge) {
	nodes := []domain.Node{}
	edges := []edge{}
	centerX := float64(width) / 2.0
	centerY := float64(height) / 2.0
	segments := []struct{ x, y, angle float64 }{
		{centerX, centerY, 0},
		{centerX, centerY, math.Pi / 2},
		{centerX, centerY, math.Pi},
		{centerX, centerY, 3 * math.Pi / 2},
	}

	for iter := 0; iter < iterations; iter++ {
		newSegments := []struct{ x, y, angle float64 }{}
		maxLength := math.Min(float64(width), float64(height)) / (2 * math.Pow(1.5, float64(iter)+1))

		for _, seg := range segments {
			length := maxLength
			endX := seg.x + length*math.Cos(seg.angle)
			endY := seg.y + length*math.Sin(seg.angle)

			// Clamp to bounds
			endX = math.Max(0, math.Min(float64(width), endX))
			endY = math.Max(0, math.Min(float64(height), endY))

			fromIdx := len(nodes)
			nodes = append(nodes, domain.Node{
				ID:        uuid.New(),
				X:         int64(seg.x),
				Y:         int64(seg.y),
				CreatedAt: time.Now(),
			})
			nodes = append(nodes, domain.Node{
				ID:        uuid.New(),
				X:         int64(endX),
				Y:         int64(endY),
				CreatedAt: time.Now(),
			})
			dist := euclideanDistance(int64(seg.x), int64(seg.y), int64(endX), int64(endY))
			edges = append(edges, edge{fromIdx, fromIdx + 1, float64(dist)})

			if r.Float64() < 0.7 {
				newSegments = append(newSegments, struct{ x, y, angle float64 }{endX, endY, seg.angle + 0.3})
				newSegments = append(newSegments, struct{ x, y, angle float64 }{endX, endY, seg.angle - 0.3})
			}
		}
		segments = newSegments
	}

	adjacency := make(map[int][]domain.Edge)
	for _, e := range edges {
		edgeObj := domain.Edge{
			ID:         uuid.New(),
			FromNodeID: nodes[e.from].ID,
			ToNodeID:   nodes[e.to].ID,
			Distance:   int64(e.dist),
			TravelTime: e.dist / 60.0,
			CreatedAt:  time.Now(),
		}
		adjacency[e.from] = append(adjacency[e.from], edgeObj)
	}

	return nodes, adjacency
}

// ============================================================================
// Algorithm 5: MST + Extra Edges (to avoid tree-shaped roads)
// ============================================================================

func MSTWithExtraEdgesMap(numNodes int, extraEdgesRatio float64, seed int64) domain.RouteGraph {
	return MSTWithExtraEdgesMapWithBounds(numNodes, extraEdgesRatio, 500, 500, seed)
}

func MSTWithExtraEdgesMapWithBounds(numNodes int, extraEdgesRatio float64, width, height int64, seed int64) domain.RouteGraph {
	r := rand.New(rand.NewSource(seed))
	nodes := generateRandomNodesWithBounds(numNodes, width, height, r)
	adjacency := mstWithExtraConnections(nodes, extraEdgesRatio, r)

	return domain.RouteGraph{
		ID:            uuid.New(),
		Name:          "MST + Extra Edges",
		Nodes:         nodes,
		AdjacencyList: adjacency,
		CreatedAt:     time.Now(),
		Seed:          seed,
		N:             numNodes,
		P:             extraEdgesRatio,
	}
}

func mstWithExtraConnections(nodes []domain.Node, extraRatio float64, r *rand.Rand) map[int][]domain.Edge {
	allEdges := []edge{}
	for i := 0; i < len(nodes); i++ {
		for j := i + 1; j < len(nodes); j++ {
			dist := euclideanDistance(nodes[i].X, nodes[i].Y, nodes[j].X, nodes[j].Y)
			allEdges = append(allEdges, edge{i, j, float64(dist)})
		}
	}

	sort.Slice(allEdges, func(i, j int) bool { return allEdges[i].dist < allEdges[j].dist })
	parent := make([]int, len(nodes))
	for i := range parent {
		parent[i] = i
	}

	var find func(int) int
	find = func(x int) int {
		if parent[x] != x {
			parent[x] = find(parent[x])
		}
		return parent[x]
	}

	mstEdges := []edge{}
	for _, e := range allEdges {
		if find(e.from) != find(e.to) {
			mstEdges = append(mstEdges, e)
			parent[find(e.from)] = find(e.to)
			if len(mstEdges) == len(nodes)-1 {
				break
			}
		}
	}

	numExtra := int(float64(len(mstEdges)) * extraRatio)
	for i := 0; i < numExtra && len(allEdges) > 0; i++ {
		idx := r.Intn(len(allEdges))
		mstEdges = append(mstEdges, allEdges[idx])
		allEdges = append(allEdges[:idx], allEdges[idx+1:]...)
	}

	adjacency := make(map[int][]domain.Edge)
	for _, e := range mstEdges {
		edgeObj := domain.Edge{
			ID:         uuid.New(),
			FromNodeID: nodes[e.from].ID,
			ToNodeID:   nodes[e.to].ID,
			Distance:   int64(e.dist),
			TravelTime: e.dist / 60.0,
			CreatedAt:  time.Now(),
		}
		revEdge := edgeObj
		revEdge.ID = uuid.New()
		revEdge.FromNodeID, revEdge.ToNodeID = revEdge.ToNodeID, revEdge.FromNodeID

		adjacency[e.from] = append(adjacency[e.from], edgeObj)
		adjacency[e.to] = append(adjacency[e.to], revEdge)
	}

	return adjacency
}

// ============================================================================
// Algorithm 6: Voronoi Graph for Realistic Route Networks
// ============================================================================

func VoronoiGraphMap(numSeeds int, gridResolution int, seed int64) domain.RouteGraph {
	return VoronoiGraphMapWithBounds(numSeeds, gridResolution, 500, 500, seed)
}

func VoronoiGraphMapWithBounds(numSeeds int, gridResolution int, width, height int64, seed int64) domain.RouteGraph {
	r := rand.New(rand.NewSource(seed))
	nodes := generateRandomNodesWithBounds(numSeeds, width, height, r)
	voronoiCells := computeVoronoiDiagramWithBounds(nodes, gridResolution, width, height)
	adjacency := voronoiToGraphWithBounds(nodes, voronoiCells, width, height)

	return domain.RouteGraph{
		ID:            uuid.New(),
		Name:          "Voronoi Graph",
		Nodes:         nodes,
		AdjacencyList: adjacency,
		CreatedAt:     time.Now(),
		Seed:          seed,
		N:             numSeeds,
		P:             0,
	}
}

func computeVoronoiDiagramWithBounds(nodes []domain.Node, resolution int, width, height int64) [][]int {
	cells := make([][]int, len(nodes))
	for i := range cells {
		cells[i] = []int{}
	}

	for x := int64(0); x < width; x += int64(resolution) {
		for y := int64(0); y < height; y += int64(resolution) {
			minDist := math.MaxFloat64
			closestSeed := 0
			for i, node := range nodes {
				dist := euclideanDistance(x, y, node.X, node.Y)
				if float64(dist) < minDist {
					minDist = float64(dist)
					closestSeed = i
				}
			}
			cells[closestSeed] = append(cells[closestSeed], int(x*height+y))
		}
	}
	return cells
}

func voronoiToGraphWithBounds(nodes []domain.Node, cells [][]int, width, height int64) map[int][]domain.Edge {
	adjacency := make(map[int][]domain.Edge)
	cellNeighbors := make(map[int]map[int]bool)

	for i := range cells {
		cellNeighbors[i] = make(map[int]bool)
	}

	for i, cellA := range cells {
		for _, coordA := range cellA {
			x1 := int64(coordA / int(height))
			y1 := int64(coordA % int(height))
			for j, cellB := range cells {
				if i == j {
					continue
				}
				for _, coordB := range cellB {
					x2 := int64(coordB / int(height))
					y2 := int64(coordB % int(height))
					if abs(x1-x2) <= 1 && abs(y1-y2) <= 1 {
						cellNeighbors[i][j] = true
					}
				}
			}
		}
	}

	for i, neighbors := range cellNeighbors {
		for j := range neighbors {
			if !cellNeighbors[j][i] {
				continue
			}
			dist := euclideanDistance(nodes[i].X, nodes[i].Y, nodes[j].X, nodes[j].Y)
			edgeObj := domain.Edge{
				ID:         uuid.New(),
				FromNodeID: nodes[i].ID,
				ToNodeID:   nodes[j].ID,
				Distance:   dist,
				TravelTime: float64(dist) / 60.0,
				CreatedAt:  time.Now(),
			}
			adjacency[i] = append(adjacency[i], edgeObj)
		}
	}

	return adjacency
}

// ============================================================================
// Map Picker Method - Select Algorithm by Number (1-6)
// ============================================================================

func PickMapByAlgorithm(algorithm int, width, height int64, seed int64) domain.RouteGraph {
	return PickMapByAlgorithmWithBounds(algorithm, width, height, seed)
}

func PickMapByAlgorithmWithBounds(algorithm int, width, height int64, seed int64) domain.RouteGraph {
	switch algorithm {
	case 1:
		return PoissonDiscSamplingMap(width, height, 50, seed, 150)

	case 2:
		return GeometricRandomGraphMapWithBounds(30, 120, width, height, seed)

	case 3:
		return ErdosRenyiSpatialMapWithBounds(30, 0.15, 200, width, height, seed)

	case 4:
		return RoadLSystemMapWithBounds(4, width, height, seed)

	case 5:
		return MSTWithExtraEdgesMapWithBounds(40, 0.25, width, height, seed)

	case 6:
		return VoronoiGraphMapWithBounds(15, 20, width, height, seed)

	default:
		return GeometricRandomGraphMapWithBounds(30, 120, width, height, seed)
	}
}

// ============================================================================
// Utility Functions
// ============================================================================

func euclideanDistance(x1, y1, x2, y2 int64) int64 {
	dx := x2 - x1
	dy := y2 - y1
	return int64(math.Sqrt(float64(dx*dx + dy*dy)))
}

func abs(x int64) int64 {
	if x < 0 {
		return -x
	}
	return x
}

func GenerateRouteGraph(name string, nodes []domain.Node, p float64, seed int64) domain.RouteGraph {
	adjacency := geometricRandomGraphConnections(nodes, p*10000)
	return domain.RouteGraph{
		ID:            uuid.New(),
		Name:          name,
		Nodes:         nodes,
		AdjacencyList: adjacency,
		CreatedAt:     time.Now(),
		Seed:          seed,
		N:             len(nodes),
		P:             p,
	}
}

func ErdosReniyiGraph(nodes []domain.Node, p float64, seed int64) map[int][]domain.Edge {
	r := rand.New(rand.NewSource(seed))
	return erdosRenyiWithConstraints(nodes, p, 100000, r)
}
