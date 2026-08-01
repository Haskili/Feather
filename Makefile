feather: decoder.cpp driver.cpp
	g++ -o feather driver.cpp

run: feather
	./feather

test: feather
	./feather -f "example.in"

clean:
	rm -f ./feather