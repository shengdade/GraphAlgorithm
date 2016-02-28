using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphAlgorithms
{
    class Node : IComparable<Node>
    {
        public int data;
        public int rank;
        public LinkedList<Node> adj;
        public Node parent;
        public Color color;
        public int d;
        public int f;
        public List<Node> children;

        public Node(int data)
        {
            this.data = data;
            this.rank = 0;
            this.adj = null;
            this.parent = null;
            this.color = Color.WHITE;
            this.d = 0;
            this.f = 0;
            this.children = null;
        }

        public int CompareTo(Node other)
        {
            if (other.d > this.d)
                return -1;
            else if (other.d == this.d)
                return 0;
            else
                return 1;
        }

        public void PrintTree(string indent, bool last)
        {
            Console.Write(indent);
            if (last)
            {
                Console.Write("\\-");
                indent += "  ";
            }
            else
            {
                Console.Write("|-");
                indent += "| ";
            }
            Console.WriteLine(data);

            for (int i = 0; i < children.Count; i++)
                children[i].PrintTree(indent, i == children.Count - 1);
        }
    }
}