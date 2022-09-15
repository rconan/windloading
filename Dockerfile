FROM rust as build

ADD Cargo.toml /test/
COPY ./src  /test/src
WORKDIR /test 

COPY modal_state_space_model_2ndOrder.zip /
ENV FEM_REPO="/"
RUN cargo build --release --bin windloading30
RUN cargo build --release --features ze60 --no-default-features --bin windloading60
RUN cargo build --release --features ze00 --no-default-features --bin windloading00

FROM rust:slim

COPY --from=build /test/target/release/windloading00 windloading00
COPY --from=build /test/target/release/windloading30 windloading30
COPY --from=build /test/target/release/windloading60 windloading60