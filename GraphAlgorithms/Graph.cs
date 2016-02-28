using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphAlgorithms
{
    class Graph
    {
        public Node[] V;
        public List<Edge> E;
        public int[,] W;
        private int time;
        //int time;

        public Graph(int n)
        {
            V = new Node[n];
            E = new List<Edge>();
            W = new int[n, n];
            for (int i = 0; i < n; i++)
            {
                V[i] = new Node(i);
                V[i].adj = new LinkedList<Node>();
                V[i].children = new List<Node>();
            }
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    if (i != j)
                        W[i, j] = int.MaxValue;
        }

        public void AddEdge(int src, int dest)
        {
            V[src].adj.AddFirst(V[dest]);
            E.Add(new Edge(V[src], V[dest]));
        }

        public void AddEdge(int src, int dest, int capacity)
        {
            V[src].adj.AddFirst(V[dest]);
            E.Add(new Edge(V[src], V[dest], capacity));
            W[src, dest] = capacity;
        }

        public void ConnectChildren()
        {
            foreach (Node v in V)
                if (v.parent != null)
                    v.parent.children.Add(v);
        }

        public void MakeUndirected()
        {
            List<Edge> originalE = new List<Edge>(E);
            foreach (Edge e in originalE)
                AddEdge(e.second.data, e.first.data, e.capacity);
        }

        public void BFS(Node startVertex)
        {
            foreach (Node u in V)
            {
                u.color = Color.WHITE;
                u.d = int.MaxValue;
                u.parent = null;
            }
            startVertex.color = Color.GRAY;
            startVertex.d = 0;
            Queue<Node> Q = new Queue<Node>();
            Q.Enqueue(startVertex);
            while (Q.Count != 0)
            {
                Node u = Q.Dequeue();
                foreach (Node v in u.adj)
                {
                    if (v.color == Color.WHITE)
                    {
                        v.color = Color.GRAY;
                        v.d = u.d + 1;
                        v.parent = u;
                        Q.Enqueue(v);
                    }
                }
                u.color = Color.BLACK;
            }
        }

        public void BFSPrint(Node startVertex)
        {
            BFS(startVertex);
            List<Node> L = new List<Node>();
            foreach (Node u in V)
            {
                L.Add(u);
            }
            L.Sort(new NodeComparerByDiscover());
            Console.Write("BFS: ");
            foreach (Node u in L)
            {
                if (u.d != int.MaxValue)
                    Console.Write(u.data + " ");
            }
            Console.WriteLine();
        }

        public void DFS()//for all vertices
        {
            foreach (Node u in V)
            {
                u.color = Color.WHITE;
                u.parent = null;
                u.d = int.MaxValue;
                u.f = 0;
            }
            time = 0;
            foreach (Node u in V)
            {
                if (u.color == Color.WHITE)
                {
                    DFSVisit(u);
                }
            }
        }

        public void DFS(Node startVertex)//for one vertex
        {
            foreach (Node u in V)
            {
                u.color = Color.WHITE;
                u.parent = null;
                u.d = int.MaxValue;
                u.f = 0;
            }
            time = 0;
            DFSVisit(startVertex);
        }

        private void DFSVisit(Node u)
        {
            time += 1;
            u.d = time;
            u.color = Color.GRAY;
            foreach (Node v in u.adj)
            {
                if (v.color == Color.WHITE)
                {
                    v.parent = u;
                    DFSVisit(v);
                }
            }
            u.color = Color.BLACK;
            time += 1;
            u.f = time;
        }//no print

        private void DFSVisitPrint(Node u)
        {
            Console.Write(" " + u.data + " ");
            time += 1;
            u.d = time;
            u.color = Color.GRAY;
            foreach (Node v in u.adj)
            {
                if (v.color == Color.WHITE)
                {
                    v.parent = u;
                    DFSVisitPrint(v);
                }
            }
            u.color = Color.BLACK;
            time += 1;
            u.f = time;
        }//with print

        public void DFSPrint()
        {
            DFS();
            List<Node> L = new List<Node>(V);
            L.Sort(new NodeComparerByDiscover());
            Console.Write("DFS: ");
            foreach (Node u in L)
            {
                Console.Write(u.data + " ");
            }
            Console.WriteLine();
        }

        public List<Node> TopologicalSort()
        {
            DFS();
            List<Node> L = new List<Node>(V);
            L.Sort(new NodeComparerByFinish());
            return L;
        }

        public Graph TransposeOfGraph()
        {
            Graph Gt = new Graph(V.Length);
            foreach (Edge edge in this.E)
                Gt.AddEdge(edge.second.data, edge.first.data, edge.capacity);
            return Gt;
        }

        public void StronglyConnectedComponents()
        {
            this.DFS();
            Graph Gt = TransposeOfGraph();
            List<Node> L = new List<Node>(V);
            L.Sort(new NodeComparerByFinish());
            foreach (Node u in Gt.V)
            {
                u.color = Color.WHITE;
                u.parent = null;
                u.d = int.MaxValue;
                u.f = 0;
            }
            time = 0;
            foreach (Node u in L)
            {
                if (Gt.V[u.data].color == Color.WHITE)
                {
                    Console.Write(" (");
                    DFSVisitPrint(Gt.V[u.data]);
                    Console.Write(") ");
                }
            }
        }

        public void MAKESET(Node x)
        {
            x.parent = x;
            x.rank = 0;
        }

        public void UNION(Node x, Node y)
        {
            LINK(FINDSET(x), FINDSET(y));
        }

        public void LINK(Node x, Node y)
        {
            if (x.rank > y.rank)
                y.parent = x;
            else
            {
                x.parent = y;
                if (x.rank == y.rank)
                    y.rank += 1;
            }
        }

        public Node FINDSET(Node x)
        {
            if (x != x.parent)
                x.parent = FINDSET(x.parent);
            return x.parent;
        }

        public List<Edge> MSTKruskal()
        {
            List<Edge> A = new List<Edge>();
            foreach (Node v in V)
                MAKESET(v);
            List<Edge> sortedE = new List<Edge>(E);
            sortedE.Sort(new EdgeComparerByWeight());
            foreach (Edge e in sortedE)
            {
                if (FINDSET(e.first) != FINDSET(e.second))
                {
                    A.Add(e);
                    UNION(e.first, e.second);
                }
            }
            return A;
        }

        public void MSTPrim(Node root)
        {
            MakeUndirected();
            foreach (Node u in V)
            {
                u.d = int.MaxValue;
                u.parent = null;
            }
            root.d = 0;
            List<Node> Q = new List<Node>(V);
            while (Q.Any())
            {
                Node u = Q.Min();
                Q.Remove(u);
                foreach (Node v in u.adj)
                {
                    if (Q.Contains(v) && EdgeWeight(u, v) < v.d)
                    {
                        v.parent = u;
                        v.d = EdgeWeight(u, v);
                    }
                }
            }
            ConnectChildren();
            root.PrintTree("", true);
            Console.Write("The minimized total weight is: " + MSTWeight(root));
        }

        public List<Edge> MSTClustering(int k)
        {
            List<Edge> A = new List<Edge>();
            foreach (Node v in V)
                MAKESET(v);
            List<Edge> sortedE = new List<Edge>(E);
            sortedE.Sort(new EdgeComparerByWeight());
            foreach (Edge e in sortedE)
            {
                if (FINDSET(e.first) != FINDSET(e.second) && A.Count + k < V.Length)
                {
                    A.Add(e);
                    UNION(e.first, e.second);
                }
            }
            List<Node> L = new List<Node>();
            foreach (Node v in V)
            {
                if (v.parent == v)
                    L.Add(v);
                else
                    v.parent.children.Add(v);
            }
            foreach (Node root in L)
            {
                Console.Write(" {");
                Console.Write(" " + root.data + " ");
                foreach (Node v in root.children)
                    Console.Write(" " + v.data + " ");
                Console.Write("} ");
            }
            return A;
        }

        public int MSTWeight(Node root)
        {
            if (root.children.Count == 0)
                return 0;
            else
            {
                int childrenWeight = 0;
                foreach (Node child in root.children)
                    childrenWeight += EdgeWeight(root, child) + MSTWeight(child);
                return childrenWeight;
            }
        }

        public int EdgeWeight(Node u, Node v)
        {
            Edge edgeFound = E.Find(x => x.first == u && x.second == v);
            return edgeFound.capacity;
        }

        public void InitializeSingleSource(Node startVertex)
        {
            foreach (Node v in V)
            {
                v.d = int.MaxValue;
                v.parent = null;
            }
            startVertex.d = 0;
        }

        public void Relax(Node u, Node v)
        {
            if (u.d != int.MaxValue && v.d > u.d + EdgeWeight(u, v))
            {
                v.d = u.d + EdgeWeight(u, v);
                v.parent = u;
            }
        }

        public Boolean BellmanFord(Node startVertex)
        {
            InitializeSingleSource(startVertex);
            for (int i = 1; i <= V.Length - 1; i++)
                foreach (Edge edge in E)
                    Relax(edge.first, edge.second);
            foreach (Edge edge in E)
                if (edge.first.d != int.MaxValue && edge.second.d > edge.first.d + EdgeWeight(edge.first, edge.second))
                    return false;
            return true;
        }

        public void DAGShortestPath(Node startVertex)
        {
            List<Node> TL = TopologicalSort();
            InitializeSingleSource(startVertex);
            foreach (Node u in TL)
                foreach (Node v in u.adj)
                    Relax(u, v);
        }

        public void Dijkstra(Node startVertex)
        {
            InitializeSingleSource(startVertex);
            List<Node> S = new List<Node>();
            List<Node> Q = new List<Node>(V);
            while (Q.Any())
            {
                Node u = Q.Min();
                Q.Remove(u);
                S.Add(u);
                foreach (Node v in u.adj)
                {
                    Relax(u, v);
                }
            }
        }

        public void BottleNeck(Node startVertex)
        {
            InitializeSingleSource(startVertex);
            List<Node> S = new List<Node>();
            List<Node> Q = new List<Node>(V);
            while (Q.Any())
            {
                Node u = Q.Min();
                Q.Remove(u);
                S.Add(u);
                foreach (Node v in u.adj)
                {
                    if (u.d != int.MaxValue && v.d > Math.Max(u.d, EdgeWeight(u, v)))
                    {
                        v.d = Math.Max(u.d, EdgeWeight(u, v));
                        v.parent = u;
                    }
                }
            }
        }

        public int[,] ExtendedShortestPaths(int[,] L, int[,] W)
        {
            int n = L.GetLength(0);
            int[,] Lp = new int[n, n];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                {
                    Lp[i, j] = int.MaxValue;
                    for (int k = 0; k < n; k++)
                        if (L[i, k] != int.MaxValue && W[k, j] != int.MaxValue)
                            Lp[i, j] = Math.Min(Lp[i, j], L[i, k] + W[k, j]);
                }
            return Lp;
        }

        public int[,] SlowAllPairsShortestPaths()
        {
            int n = W.GetLength(0);
            List<int[,]> L = new List<int[,]>();
            L.Add(W);
            for (int m = 1; m < n - 1; m++)
            {
                L.Add(ExtendedShortestPaths(L[m - 1], W));
            }
            return L[n - 2];
        }

        public int[,] FasterAllPairsShortestPaths()
        {
            int n = W.GetLength(0);
            List<int[,]> L = new List<int[,]>();
            L.Add(W);
            int m = 1;
            while (m < n - 1)
            {
                L.Add(ExtendedShortestPaths(L[m - 1], L[m - 1]));
                m = 2 * m;
            }
            return L[L.Count - 1];
        }

        public int[,] FloydWarshall()
        {
            int n = W.GetLength(0);
            List<int[,]> D = new List<int[,]>();
            D.Add(W);
            for (int k = 0; k < n; k++)
            {
                int[,] Dk = new int[n, n];
                for (int i = 0; i < n; i++)
                    for (int j = 0; j < n; j++)
                        if (D[k][i, k] != int.MaxValue && D[k][k, j] != int.MaxValue)
                            Dk[i, j] = Math.Min(D[k][i, j], D[k][i, k] + D[k][k, j]);
                        else
                            Dk[i, j] = D[k][i, j];
                D.Add(Dk);
            }
            return D[n];
        }

        public int[,] FloydWarshall2()
        {
            int n = W.GetLength(0);
            int[,] D = W;
            for (int k = 0; k < n; k++)
                for (int i = 0; i < n; i++)
                    for (int j = 0; j < n; j++)
                        if (D[i, k] != int.MaxValue && D[k, j] != int.MaxValue)
                            D[i, j] = Math.Min(D[i, j], D[i, k] + D[k, j]);
            return D;
        }

        public Boolean[,] TransitiveClosure()
        {
            int n = W.GetLength(0);
            List<Boolean[,]> T = new List<Boolean[,]>();
            Boolean[,] T0 = new Boolean[n, n];
            for (int i = 0; i < n; i++)
                T0[i, i] = true;
            foreach (Edge e in E)
                T0[e.first.data, e.second.data] = true;
            T.Add(T0);
            for (int k = 0; k < n; k++)
            {
                Boolean[,] Tk = new Boolean[n, n];
                for (int i = 0; i < n; i++)
                    for (int j = 0; j < n; j++)
                        Tk[i, j] = T[k][i, j] || (T[k][i, k] && T[k][k, j]);
                T.Add(Tk);
            }
            return T[n];
        }

        public int[,] Johnson()
        {
            Graph Gp = new Graph(V.Length + 1);
            Node s = new Node(V.Length);
            s.adj = new LinkedList<Node>();
            Gp.V[V.Length] = s;
            foreach (Edge edge in E)
                Gp.AddEdge(edge.first.data, edge.second.data, edge.capacity);
            foreach (Node v in V)
                Gp.AddEdge(s.data, v.data, 0);
            if (Gp.BellmanFord(s) == false)
            {
                Console.Write("the input graph contains a negative-weight cycle");
                return null;
            }
            else
            {
                foreach (Edge e in E)
                    e.capacity += Gp.V[e.first.data].d - Gp.V[e.second.data].d;
                int[,] D = new int[V.Length, V.Length];
                foreach (Node u in V)
                {
                    Dijkstra(u);
                    foreach (Node v in V)
                        D[u.data, v.data] = v.d + Gp.V[v.data].d - Gp.V[u.data].d;
                }
                return D;
            }
        }

        public Boolean EdgesContain(Node u, Node v)
        {
            foreach (Edge edge in E)
            {
                if (edge.Equals(u, v) == true)
                {
                    return true;
                }
            }
            return false;
        }

        public Graph ResidualNetwork()
        {
            Graph Gf = new Graph(V.Length);
            foreach (Edge edge in E)
            {
                if ((edge.capacity - edge.flow) > 0)
                    Gf.AddEdge(edge.first.data, edge.second.data, edge.capacity - edge.flow);
                if (edge.flow > 0)
                    Gf.AddEdge(edge.second.data, edge.first.data, edge.flow);
            }
            return Gf;
        }

        public int ResidualCapacity(Graph Gf, Node s, Node t)
        {
            int cf = int.MaxValue;
            while (t != s)
            {
                Edge edge = Gf.E.Find(x => x.first == t.parent && x.second == t);
                if (edge.capacity < cf)
                    cf = edge.capacity;
                t = t.parent;
            }
            return cf;
        }

        public void FordFulkerson(int s, int t)
        {
            int maxFlow = 0;
            foreach (Edge edge in E)
                edge.flow = 0;
            Graph Gf = ResidualNetwork();
            Gf.BFS(Gf.V[s]);
            //Gf.DFS(Gf.V[s]);
            while (Gf.V[t].parent != null)
            {
                int cp = ResidualCapacity(Gf, Gf.V[s], Gf.V[t]);
                while (Gf.V[t] != Gf.V[s])
                {
                    if (EdgesContain(V[Gf.V[t].parent.data], V[Gf.V[t].data]) == true)
                    {
                        Edge edge = E.Find(x => x.first == V[Gf.V[t].parent.data] && x.second == V[Gf.V[t].data]);
                        edge.flow += cp;
                        Console.WriteLine("edge (" + V[Gf.V[t].parent.data].data + "," + V[Gf.V[t].data].data + "), add " + cp);
                    }
                    else
                    {
                        Edge edge = E.Find(x => x.first == V[Gf.V[t].data] && x.second == V[Gf.V[t].parent.data]);
                        edge.flow -= cp;
                        Console.WriteLine("edge (" + V[Gf.V[t].data].data + "," + V[Gf.V[t].parent.data].data + "), minus " + cp);
                    }
                    Gf.V[t] = Gf.V[t].parent;
                }
                Gf = ResidualNetwork();
                Gf.BFS(Gf.V[s]);
                //Gf.DFS(Gf.V[s]);
            }
            foreach (Edge edge in E)
            {
                Console.WriteLine("(" + edge.first.data + "," + edge.second.data + ") -> " + edge.flow + "/" + edge.capacity);
            }
            foreach (Node v in V[s].adj)
            {
                Edge edge = E.Find(x => x.first == V[s] && x.second == v);
                maxFlow += edge.flow;
            }
            Console.Write("The maximum flow is: " + maxFlow);
        }

        public void PrintPath(Node s, Node v)
        {
            if (v == s)
            {
                Console.Write(s.data + " ");
            }
            else if (v.parent == null)
            {
                Console.Write("no path from " + s.data + " to " + v.data + " exists");
            }
            else
            {
                PrintPath(s, v.parent);
                Console.Write(v.data + " ");
            }
        }

        public void PrintAllPairsShortestPaths(Node[,] Pi, Node i, Node j)
        {
            if (i == j)
            {
                Console.Write(i.data + " ");
            }
            else if (Pi[i.data, j.data] == null)
            {
                Console.Write("no path from " + i.data + " to " + j.data + " exists");
            }
            else
            {
                PrintAllPairsShortestPaths(Pi, i, Pi[i.data, j.data]);
                Console.Write(j.data + " ");
            }
        }

        public void PrintDist()
        {
            foreach (Node v in V)
            {
                Console.WriteLine("The distance to node " + v.data + " is: " + v.d);
            }
        }

        public void PrintVertices()
        {
            foreach (Node v in V)
            {
                Console.Write(v.data + " ");
            }
            Console.WriteLine();
        }

        public void PrintEdges(List<Edge> E)
        {
            foreach (Edge edge in E)
            {
                Console.WriteLine("(" + edge.first.data + "," + edge.second.data + ")");
            }
        }

        public void PrintTable(int[,] tableToPrint)
        {
            if (tableToPrint != null)
            {
                int m = tableToPrint.GetLength(0);
                int n = tableToPrint.GetLength(1);
                for (int i = 0; i < m; i++)
                {
                    for (int j = 0; j < n; j++)
                    {
                        if (tableToPrint[i, j] != int.MaxValue)
                            Console.Write(String.Format("{0}\t", tableToPrint[i, j]));
                        else
                            Console.Write("∞\t");
                    }
                    Console.WriteLine();
                }
            }
        }

       public void PrintTable(Boolean[,] tableToPrint)
        {
            if (tableToPrint != null)
            {
                int m = tableToPrint.GetLength(0);
                int n = tableToPrint.GetLength(1);
                for (int i = 0; i < m; i++)
                {
                    for (int j = 0; j < n; j++)
                        Console.Write(String.Format("{0}\t", tableToPrint[i, j]));
                    Console.WriteLine();
                }
            }
        }
    }
}