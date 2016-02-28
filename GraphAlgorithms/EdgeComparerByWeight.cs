using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphAlgorithms
{
    class EdgeComparerByWeight : IComparer<Edge>
    {
        public int Compare(Edge x, Edge y)
        {
            if (x.capacity > y.capacity)
                return 1;
            else if (x.capacity < y.capacity)
                return -1;
            else
                return 0;
        }
    }
}