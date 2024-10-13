import unittest
import io
import sys
from graph import Graph

class TestGraph(unittest.TestCase):
    def setUp(self):
        self.g = Graph()
        self.g.init_from_file('graph.txt')

    def test_df_trav_abuja_valid(self):
        lst = Graph.df_trav_abuja();
        self.assertEqual(len(lst), 9)
        self.assertTrue('Kano' in lst)
        self.assertTrue('Kaduna' in lst)
        self.assertTrue('Abuja' in lst)
        self.assertTrue('Onitsha' in lst)
        self.assertTrue('Aba' in lst)
        self.assertTrue('Ibadan' in lst)
        self.assertTrue('Lagos' in lst)
        self.assertTrue('Benin City' in lst)
        self.assertTrue('Port Harcourt' in lst)

    def test_bf_trav_abuja_valid(self):
        lst = Graph.bf_trav_abuja();
        self.assertEqual(len(lst), 9)
        self.assertTrue('Kano' in lst)
        self.assertTrue('Kaduna' in lst)
        self.assertTrue('Abuja' in lst)
        self.assertTrue('Onitsha' in lst)
        self.assertTrue('Aba' in lst)
        self.assertTrue('Ibadan' in lst)
        self.assertTrue('Lagos' in lst)
        self.assertTrue('Benin City' in lst)
        self.assertTrue('Port Harcourt' in lst)

    def test_is_connected_true(self):
        self.assertTrue(self.g.is_connected('Lagos', 'Kaduna'))

    def test_is_connected_false(self):
        self.g.add_vertex('Jos')
        self.assertFalse(self.g.is_connected('Lagos', 'Jos'))

    def test_is_bipartite(self):
        g = Graph
        g.init_from_file('graph_bipartite.txt')
        self.assertTrue(g.is_bipartite())

    def test_is_not_bipartite(self):
        g = Graph
        g.init_from_file('graph_bipartite.txt')
        g.add_edge('A', 'C', 8)
        g.add_edge('C', 'A', 8)
        self.assertFalse(g.is_bipartite())
