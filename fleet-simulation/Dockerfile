
FROM golang:1.24 AS builder
WORKDIR /src


COPY go.mod go.sum ./
RUN go mod download


COPY . .
RUN CGO_ENABLED=0 GOOS=linux GOARCH=amd64 \
	go build -trimpath -ldflags="-s -w" -o /fleet-sim ./cmd/app


FROM scratch


COPY --from=builder /fleet-sim /fleet-sim

COPY --from=builder /etc/ssl/certs/ca-certificates.crt /etc/ssl/certs/ca-certificates.crt

EXPOSE 8080

USER 65532:65532

ENTRYPOINT ["/fleet-sim"]

