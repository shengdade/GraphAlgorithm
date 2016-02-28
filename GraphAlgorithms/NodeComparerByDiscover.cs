using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphAlgorithms
{
    class NodeComparerByDiscover : IComparer<Node>
    {
        public int Compare(Node x, Node y)
        {
            if (x.d > y.d)
                return 1;
            else if (x.d < y.d)
                return -1;
            else
                return 0;
        }
    }
}