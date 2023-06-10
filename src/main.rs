use tokio;

mod window;
mod cpu;
mod bus;
mod mem;

#[tokio::main]
async fn main() {
    window::run().await;
}
