using System;
using System.Collections.Generic;

namespace RobDrawer.PathPlanner
{
    public class BinaryTree<TItem> : IEnumerable<TItem> 
    {
        public BinaryTree<TItem> LeftTree { get; set; }
        public BinaryTree<TItem> RightTree { get; set; }

        public TItem Node {get; set;}

        public BinaryTree(TItem node)
            : this(node, null, null)
        {
        }

        public BinaryTree(TItem node, BinaryTree<TItem> left_tree, BinaryTree<TItem> right_tree)
        {
            this.Node = node;
            this.LeftTree = left_tree;
            this.RightTree = right_tree;
        }

        System.Collections.Generic.IEnumerator<TItem> System.Collections.Generic.IEnumerable<TItem>.GetEnumerator()
        {
            if (this.LeftTree != null)
            {
                foreach (TItem item in this.LeftTree)
                {
                    yield return item;
                }
            }

            yield return this.Node;

            if (this.RightTree != null)
            {
                foreach (TItem item in this.RightTree)
                {
                    yield return item;
                }
            }
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
        {
            throw new NotImplementedException();
        }
    }
}
