using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphAlgorithms
{
    class Edge
    {
        public Node first;
        public Node second;
        public int flow;
        public int capacity;

        public Edge(Node u, Node v)
        {
            this.first = u;
            this.second = v;
            flow = 0;
            capacity = 0;
        }
        public Edge(Node u, Node v, int capacityOrWeight)
        {
            this.first = u;
            this.second = v;
            flow = 0;
            this.capacity = capacityOrWeight;
        }
        public Boolean Equals(Node u, Node v)
        {
            if (this.first == u && this.second == v)
                return true;
            else
                return false;
        }
    }
}