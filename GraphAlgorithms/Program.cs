using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphAlgorithms
{
    class Program
    {
        static void Main(string[] args)
        {
            Graph G = new Graph(4);
            //The following edges are to test MST
            //G.AddEdge(0, 1, 4);
            //G.AddEdge(0, 2, 8);
            //G.AddEdge(1, 2, 11);
            //G.AddEdge(3, 2, 7);
            //G.AddEdge(3, 4, 2);
            //G.AddEdge(4, 6, 7);
            //G.AddEdge(4, 7, 4);
            //G.AddEdge(2, 5, 1);
            //G.AddEdge(5, 7, 2);
            //G.AddEdge(6, 7, 14);
            //G.AddEdge(7, 8, 10);
            //G.AddEdge(1, 4, 8);
            //G.AddEdge(3, 5, 6);
            //G.AddEdge(6, 8, 9);
            //The above edges are to test MST

            G.AddEdge(3, 0);
            G.AddEdge(1, 3);
            G.AddEdge(3, 2);
            G.AddEdge(1, 2);
            G.AddEdge(2, 1);

            //G.BFSPrint(G.V[0]);

            //G.DFSPrint();

            //G.PrintPath(G.V[0], G.V[1]);

            //List<Node> L = G.TopologicalSort();
            //G.PrintVertices();

            //G.PrintEdges();

            //G.FordFulkerson(0, 5);

            //G.Dijkstra(G.V[0]);
            //G.PrintDist();

            //G.DAGShortestPath(G.V[1]);
            //G.PrintDist();

            //Console.WriteLine(G.BellmanFord(G.V[1]));
            //G.PrintDist();

            //G.BottleNeck(G.V[0]);
            //G.PrintDist();

            //G.StronglyConnectedComponents();

            // G.MSTPrim(G.V[0]);

            //G.MSTClustering(4);

            //G.PrintTable(G.SlowAllPairsShortestPaths());

            //G.PrintTable(G.FasterAllPairsShortestPaths());

            //G.PrintTable(G.FloydWarshall());

            //G.PrintTable(G.FloydWarshall2());

            //G.PrintTable(G.Johnson());

            G.PrintTable(G.TransitiveClosure());
            //Console.Write(G.TransitiveClosure()[0, 0]);

            Console.Read();
        }
    }
}