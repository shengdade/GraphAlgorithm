using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphAlgorithms
{
    class NodeComparerByFinish : IComparer<Node>
    {
        public int Compare(Node x, Node y)
        {
            if (x.f < y.f)
                return 1;
            else if (x.f > y.f)
                return -1;
            else
                return 0;
        }
    }
}