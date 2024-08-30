#include <iostream>
#include <armadillo>

int main() {
	arma::colvec A = {{{0},{1}}};
	A.print();
	arma::mat B = {{{0,1},{1,2}}};
	B.print();

	std::cout << "A " << A.n_rows << " " << A.n_cols << ", B " << B.n_rows << " " << B.n_cols << "\n";

	arma::mat C = B*A;
	C.print();
}

